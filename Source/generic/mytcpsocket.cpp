/**
 * @file mytcpsocket.cpp
 * @brief Implementation of MyTcpSocket.
 *
 * Contains the implementation details for the MyTcpSocket class.
 */

/* Priority 1:  Serial ports, if found, then do not look futhure...
 * Priority 2:  Bluetooth, if found, then do not look futhure...
 * Priority 3:  Wlan, if found, then do not look futhure...
 * Priority 4:  Internal sensors...
 * If not found then sensor is to be disabled...
 */

// mytcpsocket.cpp
// -----------------------------------------------------------------------------
// Platform-independent interface to:
//   - Transponder (wlan, serial (USB) )
//   - Radar (wlan, serial (USB) or simulated)
//   - IMU (wlan, Built Inn, serial (USB), Bluetooth)
//   - Air Speed (wlan)
//   - Altitude (wlan)
//   - X-Plane via MQTT (for simulation inputs)
//
// Uses Qt 6.10 and some platform-specific code paths for Android / macOS / iOS.
// -----------------------------------------------------------------------------

#define UDP     // TODO: Clarify usage or remove if unused

#include <QTime>
#include <QTimer>
#include <QThread>
#include <QHostAddress>
#include <QNetworkInterface>

#include <QStringList>

#ifdef Q_OS_ANDROID
#include <QtCore/private/qandroidextras_p.h>
#include <QJniObject>
#endif

#ifdef Q_OS_IOS
//   #include "IOS_swift/WitSDK/Sensor/Modular/Processor/Roles/BWT901BLE5_0DataProcessor.swift"
#undef Q_OS_MAC
#endif

#ifdef Q_OS_MAC
#include <QSerialPort>
#endif

#include <QList>
#include <QCoreApplication>
#ifndef Q_OS_IOS
#include <QSerialPortInfo>
#endif
#include <QDebug>

#include "mytcpsocket.h"
#include "wit_c_sdk.h"
#include "tcpclient.h"

#include <QVector>
#include <QString>
#include <QMap>
#include <unistd.h>   // usleep
#include "REG.h"
#include "ssdp.h"
//#include "multicastlistner.h"

#ifndef USE_BT_IMU
    #define ComBt void
#endif
/*
#ifndef ComBt
    #define ComBt nullptr_t
#endif
*/

// Enable internal radar simulator (used when real radar is not present)
#undef SIMULATE_RADAR
typedef struct { double L, F, D; } LFD;

// External C interfaces (implemented elsewhere)
extern bool INS_driver(void *handler, ComQt *serPorts, ComBt *serPortb, void *func);
extern bool AutoScanSensor();
extern void AutoSetBaud(int);

// ============================================================================
// Constructor / Destructor
// ============================================================================

/**
 * @brief Construct a MyTcpSocket object.
 *
 * @param parent   QObject parent (Qt ownership).
 * @param s        Pointer to log / debug text output widget.
 * @param retx     Callback invoked by serial ports (transponder) when data arrives.
 * @param rety     Callback used to inform about IMU connection status.
 */
MyTcpSocket::MyTcpSocket(QObject *parent,
                         QPlainTextEdit *s,
                         void (*retx)(void *, const char *, uint32_t),
                         void (*rety)(void *, bool use_imu))
    : QObject(parent)
{
    this->ret_transponder   = retx;  // C callback for transponder data
    this->ret_imu  = rety;  // C callback to notify IMU status
    this->text     = s;
    this->parent   = parent;

    // ---------------------------------------------------------------------
    // Serial / Bluetooth COM objects
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS

    // Transponder serial port
    TransponderSerPort = new ComQt(parent);
    TransponderSerPort->setParent(this);
    TransponderSerPort->setRxCallback(ret_transponder);             // register C callback

    // Radar serial port
    RadarSerPort = new ComQt(this);
    RadarSerPort->setParent(this);
    RadarSerPort->setRxCallback(doRadar);               // static callback -> MyTcpSocket::doRadar

    // INS / IMU serial port
    INSSerPort = new ComQt(parent);
    INSSerPort->setParent(this);
    INSSerPort->setRxCallback(WitSerialDataIn);         // callback from WIT C SDK

    // INS / IMU serial port
    AltimeterPort = new ComQt(this);
    AltimeterPort->setParent(this);
    AltimeterPort->setRxCallback(ret_altimeter);         // callback from WIT C SDK

    // INS / IMU serial port
    AirSpeedPort = new ComQt(this);
    AirSpeedPort->setParent(this);
    AirSpeedPort->setRxCallback(ret_airspeed);         // callback from WIT C SDK

    // Bluetooth IMU port
 #if defined(USE_BT_IMU)
    bluetootPort = new ComBt(this);
    bluetootPort->setParent(this);
    bluetootPort->setRxCallback(WitSerialDataIn);       // callback from WIT C SDK
 #endif

    qDebug() << "Starting requester...";
    timerTRANS = new QTimer(this);
    timerTRANS->setSingleShot(false);
    connect(timerTRANS, SIGNAL(timeout()), SLOT(doTransponder()));
    timerTRANS->start(200);

    // Build a serial-number -> port map on macOS
    map = serialToPortMap(true);
    qDebug() << map;

#endif

    //-------------------------------------------------------------
    // Set up the multicast listern to find wlan sensors...
    ssdpConfig();

    // ---------------------------------------------------------------------
    // MQTT setup (X-Plane / simulator input)
    // ---------------------------------------------------------------------
    // NOTE: Currently configured for localhost mosquitto / similar broker.
//    SERVER_ADDRESS = std::string("tcp://localhost:1883");
    SERVER_ADDRESS = std::string("tcp://172.20.10.3:1883");
    CLIENT_ID      = std::string("transponder");

    mqtt = new MqttClient(SERVER_ADDRESS, CLIENT_ID);
    mqtt->setMessageHandler([this](const std::string &topic, const std::string &payload) {
        try {
            this->handleUpdate(topic, payload);
        } catch (const std::exception &e) {
            qWarning() << "Invalid float in payload:"
                       << QString::fromStdString(payload)
                       << "Error:" << e.what();
        }
    });    

    mqtt->connect();
    mqtt->subscribe("xplane/+");
//    mqtt->subscribe("xplane/#");
    try {
        mqtt->sendMessage("xplane/topic", "1.0 eMove GUI Controller!");
    } catch (const mqtt::exception &e) {
        printf("MQTT publish error: %s\n", e.what());
    }

    // ---------------------------------------------------------------------
    // Deferred startup using a timer (ensures constructor returns first)
    // ---------------------------------------------------------------------
    timerStart = new QTimer(this);
//    timerStart->setSingleShot(false);
    connect(timerStart, SIGNAL(timeout()), this, SLOT(doStart()));
    timerStart->start(500);                             // first step after 200 ms

}

/**
 * @brief Destructor. Closes all serial ports.
 */
MyTcpSocket::~MyTcpSocket()
{
#ifndef Q_OS_IOS
    TransponderSerPort->close();
    RadarSerPort->close();
    INSSerPort->close();
#endif
    qDebug() << "Stopped socket...";
}

/**
 * @brief multicastConfig. Sends multicast...
 */
void MyTcpSocket::ssdpConfig()
{
    disc = new SsdpDiscoverer(this);

    connect(disc, &SsdpDiscoverer::deviceFound,
            this, [this](const QHostAddress &addr, quint16 port, const QString &st)
            {
                (void) port;
        /*
                qDebug() << "SSDP device found:"
                         << "IP="  << addr.toString()
                         << "PORT="<< QString::number(port)
                         << "ST="  << st;
        */
                // ---------- Match device type by ST ----------
                int pos = st.indexOf('-');
                QString usn = st;
                if (pos != -1) {
                    usn = st.left(pos);
                }

                if (usn == "IMU") {
                    if(m_imu_address == ""){
                        m_imu_address = addr.toString();
                        qDebug() << "IMU FOUND:" << m_imu_address;
                        connectedIMUWlan();
                    }
                }
                else if (usn == "RADAR") {
                    if(m_radar_address == ""){
                        m_radar_address = addr.toString();
                        qDebug() << "RADAR FOUND:" << m_radar_address;
                        connectedRadarWlan();
                    }
                }
                else if (usn == "T2000U") {
                    if(m_transponder_address == ""){
                        m_transponder_address = addr.toString();
                        qDebug() << "TRANSPONDER FOUND:" << m_transponder_address;
                        transponderConnect();
                    }
                }
                else if (usn == "ALTIMETER") {
                    if(m_altimeter_address == ""){
                        m_altimeter_address = addr.toString();
                        qDebug() << "ALTIMETER FOUND:" << m_altimeter_address;
                        connectedAltitude();
                    }
                }
                else if (usn == "AIRSPEED") {
                    if(m_airspeed_address == ""){
                        m_airspeed_address = addr.toString();
                        qDebug() << "AIRSPEED FOUND:" << m_airspeed_address;
                        connectedAirspeed();
                    }
                }
                else {
                    qDebug() << "Unknown SSDP service type:" << usn;
                }
            });
}

// ============================================================================
// macOS specific helpers (serial port enumeration & mapping)
// ============================================================================

//#ifdef Q_OS_MAC

/**
 * @brief Enumerate serial ports with extended metadata.
 *
 * @return QVector of PortEntry (one per detected port).
 */
QVector<PortEntry> MyTcpSocket::listSerialPortsDetailed()
{
    QVector<PortEntry> out;
#ifndef Q_OS_IOS
    const auto ports = QSerialPortInfo::availablePorts();
    out.reserve(ports.size());

    for (const QSerialPortInfo &p : ports) {
        PortEntry e;
        e.serial         = p.serialNumber();   // may be empty on some adapters/OSes
        e.portName       = p.portName();
        e.systemLocation = p.systemLocation();
        e.description    = p.description();
        e.manufacturer   = p.manufacturer();
        e.vendorId       = p.hasVendorIdentifier()  ? p.vendorIdentifier()  : 0;
        e.productId      = p.hasProductIdentifier() ? p.productIdentifier() : 0;
        out.push_back(e);
    }
#endif
    return out;
}

/**
 * @brief Build a map from serial number -> device path / port name.
 *
 * @param useSystemLocation If true, map to systemLocation; otherwise to portName.
 * @return QMap<serialNumber, portPath>
 */
QString portNum ="";  // reused temporary
static void ret_test(void *userData, const char *data, uint32_t size)
{
    Q_UNUSED(userData);
    QByteArray bytes(data, static_cast<int>(size));
    QString response = QString::fromUtf8(data).trimmed();
    if (response.contains("T2000U")) {
        portNum = "Transponder";
    }
    if (response.contains("AIRSPEED")) {
        portNum = "Airspeed";
    }
    if (response.contains("ALTIMETER")) {
        portNum = "Altitude";
    }
    if (response.contains("RADAR")) {
        portNum = "Radar";
    }
    if (response.contains("IMU")) {
        portNum = "Imu";
    }
    qDebug() << "Found device: " << portNum;
}



/**
 * @brief Build a map from serial number -> device path / port name.
 *
 * @param useSystemLocation If true, map to systemLocation; otherwise to portName.
 * @return QMap<serialNumber, portPath>
 */
#ifdef ANDROID_x
QMap<QString, QString> MyTcpSocket::serialToPortMap(bool useSystemLocation)
{
    static QString portNum = "";  // reused temporary

    qDebug() << "Looking for serial ports....";
    QMap<QString, QString> result;
    static const auto &x= listSerialPortsDetailed();
    for (const auto &e : listSerialPortsDetailed()) {
        // If FTDI og Profillic...
        qSetMessagePattern("%{time HH:mm:ss.zzz} %{type}: %{message}");
        qDebug() << "SerialPort:" << e.systemLocation << e.portName << e.serial << e.productId << e.vendorId;


        if (!e.portName.isEmpty()){ // && (!e.serial.isEmpty() || e.vendorId > 100)) {

            if(e.serial.isEmpty())
                portNum =  QString::number(e.productId);
            else
                portNum = e.serial;

            result.insert(portNum, useSystemLocation ? e.systemLocation : e.portName);
        }
    }
    qDebug() << result;
    return result;
}
#else


QMap<QString, QString> MyTcpSocket::serialToPortMap(bool useSystemLocation)
{
    QMap<QString, QString> result;
    useconds_t us = (useconds_t)500 * 1000u;
    int i = 0;

#ifndef Q_OS_IOS
    // Wait for the devices to boot...
    usleep(us);

    for (const auto &e : listSerialPortsDetailed()) {
        portNum ="";

        qSetMessagePattern("%{time HH:mm:ss.zzz} %{type}: %{message}");
        qDebug() << "SerialPort:" << e.systemLocation << e.portName << e.serial << e.productId << e.vendorId;

        // If FTDI og Profillic...
        if (!e.portName.isEmpty()){  // && (!e.serial.isEmpty())){  // || e.vendorId > 100)) {

            // Probe port: open + close to check availability
            ComQt *TestSerPort = new ComQt();
            TestSerPort->setParent(this);
            TestSerPort->setRxCallback(ret_test);             // register C callback

#ifdef Q_OS_ANDROID   // Only Android version has the Java backlight hook
            if (TestSerPort->open(QString::number(i), QSerialPort::Baud9600)){
#else
            if (TestSerPort->open(e.systemLocation, QSerialPort::Baud9600)){
#endif
                TestSerPort->send("\x02" "z=?" "\x03");
                usleep(us);
                TestSerPort->send("\x02" "z=?" "\x03");
                usleep(us);
                TestSerPort->send("\x02" "z=?" "\x03");
                usleep(us);
                TestSerPort->send("\x02" "z=?" "\x03");
                usleep(us);

                // NÅ fungere Android men ikk MAC .....
            }

            TestSerPort->close();
            delete TestSerPort;

//            if(portNum == ""){
//                if(e.serial.isEmpty()) portNum =  QString::number(e.productId);
//                else                   portNum = e.serial;
//            }
#ifdef Q_OS_ANDROID   // Only Android version has the Java backlight hook
            if(portNum != ""){
                result.insert(portNum, QString::number(i));
            }
            i++;
#else
            result.insert(portNum, useSystemLocation ? e.systemLocation : e.portName);
#endif
        }
    }
#endif
    qDebug() << result;
    return result;
}
#endif

/**
 * @brief Find the serial port path associated with a given device serial number.
 *
 * @param targetSerial Target serial number.
 * @return System location / port name, or empty string if not found.
 */
QString MyTcpSocket::findPort(QString targetSerial)
{
    if (map.contains(targetSerial)) {
        return map.value(targetSerial);
    } else {
        qWarning() << "Device with serial" << targetSerial << "not found";
    }
    return "";
}
//#endif  // Q_OS_MAC

// ============================================================================
// Startup state machine (runs on timerStart)
// ============================================================================

/**
 * @brief Progressive startup handler.
 *
 * State machine driven by timerStart:
 *   state 0: Try connect IMU (BT / serial).
 *   state 1: Try connect transponder (USB).
 *   state 2: Keep monitoring / retry transponder.
 * Also optionally generates radar/IMU simulation data when SIMULATE_RADAR is set.
 */
void MyTcpSocket::doStart()
{
    static int delay = 0;
    static int send_boot = 10;

    static int state = 0;
    timerStart->stop();

    switch(state){
    case 0:
        disc->startDiscovery(5000);  // listen for 5 seconds
        // Wait for a total of 2.5 seconds...
        if(send_boot++ > 20){
            send_boot = 0;
            ++state;
        }
        QCoreApplication::processEvents();
        break;
    case 1:
        connectedAltitudeSerial();     // try to find & initialize Altimeter
        ++state;
        break;
    case 2:
        connectedAirSpeedSerial();
        ++state;
        break;
    case 3:
        connectedRadar();          // try to find & initialize IMU / INS
        ++state;
        break;
    case 4:
        connected();             // try to find & initialize transponder
        ++state;
        break;
    case 5:
        connectedIMU();          // try to find & initialize IMU / INS
        ++state;
        break;
    case 6:
        if(delay++ > 15){
            delay = 0;
            setbacklit();     // Android: periodically force bright backlight
        }
        /*
        switch(delay){
        case 5:
            if(!IMUconnected){
                connectedIMU();          // try to find & initialize IMU / INS
            }
        case 10:
            if(!Radarstat){
                connectedRadar();          // try to find & initialize IMU / INS
            }
        case 14:
            if (!Transponderstat){
                connected();
            }
        }
        */
        break;
    }

    // ---------------------------------------------------------------------
    // Radar / IMU simulation (for debugging without hardware)
    // ---------------------------------------------------------------------
#ifdef SIMULATE_RADAR
    static float simX = 0.0f;
    static float simZ = 0.0f;
    char buffer[512];

    simX += 0.4f;
    simZ += 0.2f;
    if (simX > (100.0f / 1.414f)) simX = 0.0f;
    if (simZ > (100.0f / 0.707f)) simZ = 0.0f;

    // Simulated radar payload format: "speed,altX,altZ"
    snprintf(buffer, sizeof(buffer), "123.4,%f,%f", simX, simZ);
    doRadar(this, buffer, strlen(buffer));
    Radarstat = true;

    timerStart->start(100);   // faster timer while simulating

    static float t0 = 0.0f;
    float t = t0;
    t0 += 0.1f;               // 0.1 s step

    // Simulated attitude angles
    float roll  = 30.0f * sinf(2.0f * 3.14159f * 0.2f * t);
    float pitch = 20.0f * sinf(2.0f * 3.14159f * 0.1f * t);
    float yaw   = fmodf(t * 20.0f, 360.0f) - 180.0f;

    AngleX = roll;
    AngleY = pitch;
    AngleZ = yaw;
    IMUconnected = true;
    Transponderstat = true;
#endif  // SIMULATE_RADAR

    timerStart->start(500);
}

// ============================================================================
// Android display backlight helper
// ============================================================================

/**
 * @brief On Android, periodically set the external transponder display backlight.
 *
 * Uses Java class: com.hoho.android.usbserial.driver.TestClassTerje
 */
void MyTcpSocket::setbacklit()
{
#ifdef Q_OS_ANDROID   // Only Android version has the Java backlight hook
    static int disp = 999;

    if (someJavaObject == nullptr) {
        QJniEnvironment env;
        auto context = QJniObject(QNativeInterface::QAndroidApplication::context());

        // Check if Java class is available and construct helper object
        if (QJniObject::isClassAvailable(
                "com/hoho/android/usbserial/driver/TestClassTerje")) {
            someJavaObject = new QJniObject(
                "com/hoho/android/usbserial/driver/TestClassTerje",
                "(Landroid/content/Context;)V",
                context.object());
        }
    }

    // Every 40 ticks we send a "change brightness" command
    if (++disp > 40 && someJavaObject != nullptr) {
        disp = 0;
        int y = someJavaObject->callMethod<jint>("change", "(I)I", 255);
        qDebug() << "Display Backlit set to:" << y;
    }
#endif
}

// ============================================================================
// IMU / INS / Radar connection setup
// ============================================================================

/**
 * @brief Try to connect to IMU/INS and Radar devices.
 *
 * Order:
 *   1. Bluetooth IMU (WT901 BLE) if enabled.
 *   2. Serial IMU WTGAHRS1/3.
 *   3. Wlan IMU WTGAHRS1/3.
 */
void MyTcpSocket::connectedIMU()
{
#ifndef Q_OS_ANDROID
    // callbacks = new Callbacks();   // Left as a placeholder if needed
#endif

    // ---------------------------------------------------------------------
    // 1) Bluetooth IMU (WT901BLE67), if compiled with USE_BT_IMU
    // ---------------------------------------------------------------------
#if defined(USE_BT_IMU)
/*
    NoButtonMessageBox *m_msgBoxIMU = new NoButtonMessageBox(
        tr("Looking for Bluetooth device WT901BLE67 ..."));
    m_msgBoxIMU->show();
    QCoreApplication::processEvents();
    QThread::msleep(2000);
    m_msgBoxIMU->hide();
    delete m_msgBoxIMU;
*/
    // Wait for BT scan to finish
    int timeout = 5*10;
    while (!bluetootPort->serial_->scancomplete) {
        QThread::msleep(200);
        QCoreApplication::processEvents();
        if(--timeout == 0) break;
    }

    if (bluetootPort->open("", 0)) {
        // Launch INS driver in BT mode

        INS_driver(static_cast<void *>(this),
                   static_cast<ComQt *>(nullptr),
                   bluetootPort,
                   reinterpret_cast<void *>(parseIMU));

     //   IMUconnected = false;
        for (int delay = 0; delay < 8; ++delay) {
            QThread::msleep(200);
            QCoreApplication::processEvents();

            if (AutoScanSensor()) {
                IMUconnected = true;
                break;
            }
        }
    } else {
        qDebug() << "IMU NOT Connected (Bluetooth)...";
     //   IMUconnected = false;
    }

#endif  // USE_BT_IMU

    // ---------------------------------------------------------------------
    // 2) USB IMU / INS device WTGAHRS3 (serial)
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS
    if (!IMUconnected)
    {
        QString IMU_name = findPort(_IMU_copy);
        qDebug() << "Looking for Port:" << IMU_name;
        if (!IMU_name.isEmpty() && INSSerPort->open(IMU_name, QSerialPort::Baud9600))
        {
            // Launch INS driver in serial mode
            INS_driver(static_cast<void *>(this), INSSerPort, nullptr,reinterpret_cast<void *>(parseIMU));

            for (int delay = 0; delay < 10; ++delay) {
                if (AutoScanSensor()) {
                    // Optionally change baud rate to 115200 once recognized
                    // AutoSetBaud(QSerialPort::Baud115200);
                    IMUconnected = true;
                    break;
                }
                QThread::msleep(100);
                QCoreApplication::processEvents();
                if (delay == 9) {
                 //   IMUconnected = false;
                }
                if (delay == 5) {
                    // AutoSetBaud(QSerialPort::Baud115200);
                }
            }
        }
    }
#endif  // !Q_OS_IOS
    // Notify IMU connection state through callback
    this->ret_imu(this->parent, IMUconnected);

}

/**
 * @brief Try to connect to IMU/INS and Radar devices.
 *
 * Order:
 *   1. Bluetooth IMU (WT901 BLE) if enabled.
 *   2. Serial IMU WTGAHRS1/3.
 *   3. Wlan IMU WTGAHRS1/3.
 */
void MyTcpSocket::connectedIMUWlan()
{
    // --------------------------------------------------------------------------------
    // Check is the IMU sensor in connected on the wlan...
 //   if(IMUconnected == false)
    {
        if(m_imu_address != "")
        {
            // somewhere in ctor or init:
          //  static void*_this = this;

            if(m_imuClient) delete(m_imuClient);
            m_imuClient = new QTcpSocket(this);
            connect(m_imuClient, &QTcpSocket::connected, this, [this]() {
                qDebug() << "IMU TCP connected";
                IMUconnected = true;
//                INS_driver(static_cast<void *>(this), nullptr, nullptr,reinterpret_cast<void *>(parseIMU));
            });
            connect(m_imuClient, &QTcpSocket::disconnected, this, [this]() {
                (void) this;
                qDebug() << "IMU TCP disconnected";
         //       IMUconnected = false;
            });
            connect(m_imuClient, &QTcpSocket::readyRead, this, [this]() {
                QByteArray data = m_imuClient->readAll();
                if (!data.isEmpty()) {
                    WitSerialDataIn(this, data.constData(), data.size());
                }
            });

            m_imuClient->connectToHost(QHostAddress(m_imu_address), 23);
            INS_driver(static_cast<void *>(this), nullptr, nullptr,reinterpret_cast<void *>(parseIMU));
        }
    }
}

void MyTcpSocket::connectedRadar()
{
    // ---------------------------------------------------------------------
    // 3) Radar (USB serial and NET)
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS
    QString radar_name = findPort(_radar_copy);
    qDebug() << "Looking for Radar Port:" << radar_name;
    if (!radar_name.isEmpty() && RadarSerPort->open(radar_name, QSerialPort::Baud115200)) {
        Radarstat = true;
    }
#endif
}

void MyTcpSocket::connectedRadarWlan()
{
 //   if (!Radarstat)
    {
        if(m_radar_address != "")
        {
            // somewhere in ctor or init:
        //    static void*_this = this;

            if(m_radarClient) delete(m_radarClient);
            m_radarClient = new QTcpSocket(this);
            connect(m_radarClient, &QTcpSocket::connected, this, [this]() {
                qDebug() << "Radar TCP connected";
                Radarstat = true;
            });
            connect(m_radarClient, &QTcpSocket::disconnected, this, [this]() {
                qDebug() << "Radar TCP disconnected";
                Radarstat = false;
            });
            connect(m_radarClient, &QTcpSocket::readyRead, this, [this]() {
                QByteArray data = m_radarClient->readAll();
                if (!data.isEmpty()) {
                    doRadar(this, data.constData(), data.size());
                }
            });
            m_radarClient->connectToHost(QHostAddress(m_radar_address), 23);
        }
    }
}

// ============================================================================
// Transponder connection
// ============================================================================

/**
 * @brief Try to connect transponder on the configured serial port.
 *
 * On success:
 *   - Sets Transponderstat = true
 *   - Queries version and configuration
 *   - Starts timerTRANS to periodically call doTransponder()
 */
void MyTcpSocket::connected()
{
#ifndef Q_OS_IOS
//    if (TransponderSerPort->open("0", QSerialPort::Baud9600))
    QString transponder_name = findPort(_transponder_copy);
    if (!transponder_name.isEmpty() &&
        TransponderSerPort->open(transponder_name, QSerialPort::Baud9600))
    {
        Transponderstat = true;
    }
#endif
}

    /**
 * @brief Try to connect transponder on the configured serial port.
 *
 * On success:
 *   - Sets Transponderstat = true
 *   - Queries version and configuration
 *   - Starts timerTRANS to periodically call doTransponder()
 */
void MyTcpSocket::transponderConnect()
{
 //   if (!Transponderstat)
    {
        if(m_transponder_address != "" && Transponderstat == false)
        {
            // somewhere in ctor or init:
          //  static void*_this = this;
            if(m_transponderClient) delete(m_transponderClient);
            m_transponderClient = new QTcpSocket(this);
            connect(m_transponderClient, &QTcpSocket::connected, this, [this]() {
                qDebug() << "Transponder TCP connected";
                Transponderstat = true;
            });
            connect(m_transponderClient, &QTcpSocket::disconnected, this, [this]() {
                qDebug() << "Transponder TCP disconnected";
                Transponderstat = false;
            });
            connect(m_transponderClient, &QTcpSocket::readyRead, this, [this]() {
                Transponderstat = true;
                QByteArray data = m_transponderClient->readAll();
                if (!data.isEmpty()) {
                    this->ret_transponder(this->parent, data, data.length());
                }
            });
            m_transponderClient->connectToHost(QHostAddress(m_transponder_address), 23);

        }
    }
}


/**
 * @brief Try to connect transponder on the configured serial port.
 *
 * On success:
 *   - Sets Transponderstat = true
 *   - Queries version and configuration
 *   - Starts timerTRANS to periodically call doTransponder()
 */
void MyTcpSocket::connectedAltitude()
{
 //   if (!Altitudestat)
    {
        if(m_altimeter_address != "")
        {
            // somewhere in ctor or init:
          //  static void*_this = this;
            if(m_altimeterClient) delete(m_altimeterClient);
            m_altimeterClient = new QTcpSocket(this);
            connect(m_altimeterClient, &QTcpSocket::connected, this, [this]() {
                qDebug() << "Altimeter TCP connected";
                Altitudestat = true;
            });
            connect(m_altimeterClient, &QTcpSocket::disconnected, this, [this]() {
                qDebug() << "Altimeter TCP disconnected";
                Altitudestat = false;
            });
            connect(m_altimeterClient, &QTcpSocket::readyRead, this, [this]() {
                QByteArray data = m_altimeterClient->readAll();
                if (!data.isEmpty()) {
                    this->parseAltimeterLine(this,data);
                }
            });
            m_altimeterClient->connectToHost(QHostAddress(m_altimeter_address), 23);
        }
    }
}

void MyTcpSocket::connectedAltitudeSerial()
{
#ifndef Q_OS_IOS
    QString port_name = findPort(_Altitude_copy);
    qDebug() << "Looking for Altimeter Port:" << port_name;
    if (!port_name.isEmpty() &&
        AltimeterPort->open(port_name, QSerialPort::Baud115200)) {
        Altitudestat = true;
    }
#endif
}

/// Callback invoked when serial data arrives from transponder.
void MyTcpSocket::ret_altimeter(void *host, const char *data, uint32_t size){
    MyTcpSocket *thiz = (MyTcpSocket*)host;
    QString text = QString::fromLatin1(data, static_cast<qsizetype>(size));
    thiz->parseAltimeterLine(thiz,text);
}

void MyTcpSocket::connectedAirSpeedSerial()
{
#ifndef Q_OS_IOS
    QString port_name = findPort(_AirSpeed_copy);
    qDebug() << "Looking for AirSpeed Port:" << port_name;
    if (!port_name.isEmpty() && AirSpeedPort->open(port_name, QSerialPort::Baud115200)) {
        Airspeedstat = true;
    }
#endif
}

/// Callback invoked when serial data arrives from transponder.
void MyTcpSocket::ret_airspeed(void *host, const char *data, uint32_t size){
    MyTcpSocket *thiz = (MyTcpSocket*)host;
    QString text = QString::fromLatin1(data, static_cast<qsizetype>(size));
    thiz->parseAirspeedLine(thiz,text);
}

/**
 * @brief Try to connect transponder on the configured serial port.
 *
 * On success:
 *   - Sets Transponderstat = true
 *   - Queries version and configuration
 *   - Starts timerTRANS to periodically call doTransponder()
 */
void MyTcpSocket::connectedAirspeed()
{
 //   if (!Airspeedstat)
    {
        if(m_airspeed_address != "")
        {
            // somewhere in ctor or init:
         //   static void*_this = this;
            if(m_airspeedClient) delete(m_airspeedClient);
            m_airspeedClient = new QTcpSocket(this);
            connect(m_airspeedClient, &QTcpSocket::connected, this, [this]() {
                qDebug() << "Airspeed TCP connected";
                Airspeedstat = true;
            });
            connect(m_airspeedClient, &QTcpSocket::disconnected, this, [this]() {
                qDebug() << "Airspeed TCP disconnected";
                Airspeedstat = false;
            });
            connect(m_airspeedClient, &QTcpSocket::readyRead, this, [this]() {
                QByteArray data = m_airspeedClient->readAll();
                if (!data.isEmpty()) {
                    this->parseAirspeedLine(this,data);
                }
            });
            m_airspeedClient->connectToHost(QHostAddress(m_airspeed_address), 23);
        }
    }
}

// ============================================================================
// IMU data handling
// ============================================================================

/**
 * @brief Static callback from INS_driver / WIT SDK.
 *
 * Parses ASCII key/value pairs such as:
 *   "AccX 0.123", "AngleY 10.0", "LAT 59.00", ...
 *
 * @param parent  Pointer back to MyTcpSocket instance (this).
 * @param data    Null-terminated ASCII string from the IMU.
 * @param length  Data length in bytes (unused here).
 */

static inline int32_t join32(uint16_t lo, uint16_t hi) {
    return (int32_t)(((uint32_t)hi << 16) | lo);
}

static double nmea_ddmm_to_deg(int32_t raw) {
    // raw is ddmm.mmmmmm with decimal removed; may be signed
    int sign = (raw < 0) ? -1 : 1;
    uint32_t v = (raw < 0) ? (uint32_t)(-raw) : (uint32_t)raw;

    uint32_t dd        = v / 10000000U;       // whole degrees
    uint32_t mm_x1e6   = v % 10000000U;       // minutes * 1e6
    double minutes     = mm_x1e6 / 100000.0;  // mm.mmmmmm

    double deg = (double)dd + minutes / 60.0;
    return sign * deg;
}

void MyTcpSocket::parseIMU(void *parent,uint32_t uiReg, uint16_t sRegAll[])
{
    (void) sRegAll;
    auto *local = static_cast<MyTcpSocket *>(parent);

    if(uiReg == Roll)
    {
        const double   g     = 9.82500;

        local->AccX        = -1*((float)sReg[AX   + 0] / 32768.0f * 16.0f) * g;
        local->AccY        =    ((float)sReg[AX   + 1] / 32768.0f * 16.0f) * g;
        local->AccZ        =    ((float)sReg[AX   + 2] / 32768.0f * 16.0f) * g;

        local->AsZ         =    ((float)sReg[GX   + 0] / 32768.0f * 2000.0f);
        local->AsY         = -1*((float)sReg[GX   + 1] / 32768.0f * 2000.0f);
        local->AsX         =    ((float)sReg[GX   + 2] / 32768.0f * 2000.0f);

        local->AngleX      =    ((float)sReg[Roll + 0] / 32768.0f * 180.0f); //-90.0;
        local->AngleY      =    ((float)sReg[Roll + 1] / 32768.0f * 180.0f);
        local->AngleZ      = -1*((float)sReg[Roll + 2] / 32768.0f * 180.0f);

        local->HX          =     (float)sReg[HXi  + 0];
        local->HY          =     (float)sReg[HXi  + 1];
        local->HZ          =     (float)sReg[HXi  + 2];

        // GPS / altitude
        local->m_longitude = nmea_ddmm_to_deg(join32( sReg[LonL],sReg[LonH]));
        local->m_latitude = nmea_ddmm_to_deg(join32(sReg[LatL],sReg[LatH]));
        //        GPS[2] = (float)sReg[GPSHeight]/10.0; // Get altitude...
        local->m_altitude = (float)sReg[D0Status]/10.0; // Get altitude...

        local->Temp = (float) sReg[TEMP]/100.0;
        local->VER = sReg[VERSION];

        // If we got a seperat pressure decoder then do NOT use the IMU pressure...
        if(!local->Altitudestat){
            local->m_pressure_raw = join32(sReg[PressureL],sReg[PressureH])/100.0;
          //  local->m_preasure_alt     = join32(sReg[HeightL],sReg[HeightH])/100.0;
        }

        // Find yaw angle...
        /*
        auto wrap360 = [](double deg) {
            deg = fmod(deg, 360.0);
            if (deg < 0) deg += 360.0;
            return deg;
        };
        */
        // Treat registers as UNSIGNED 16-bit before joining
        uint32_t raw100 = (uint32_t)(uint16_t)sReg[GPSVL]
                          | ((uint32_t)(uint16_t)sReg[GPSVH] << 16);

        // Now convert to float speed (whatever unit you encoded)
        float speed = raw100 / 100.0f;

        local->Donwn_Speed = 0;
        local->m_gpsspeed  = speed;
        local->m_speed     = speed; //abs(vel.F);
       // qDebug() << "GPSVL=" << local->m_speed;
    }
}

// ============================================================================
// Radar data handling
// ============================================================================

/**
 * @brief Static callback for Radar serial data.
 *
 * Expected data format: "pos,radialSpeed,radialDist"
 * Performs simple geometry using a fixed azimuth.
 *
 * @param parent  Pointer back to MyTcpSocket instance.
 * @param data    Null-terminated ASCII CSV.
 * @param length  Data length.
 */
void MyTcpSocket::doRadar(void *parent, const char *data, uint32_t length)
{
    // Fixed radar azimuth (degrees)
    constexpr float kAzimuthDeg = 0.0f;  // 45.0

    auto *local = static_cast<MyTcpSocket *>(parent);
    if (length > 5) {
        QString str = QString::fromLatin1(data);
     //   const QList<QString> fields = str.split(',');
        QStringList fields = str.split(QRegularExpression("[,\n]"),Qt::SkipEmptyParts);

        if (fields.length() >= 4) {
            const float azimuthRad = kAzimuthDeg / (180.0f / static_cast<float>(M_PI));
        //    const float cosAz      = cosf(azimuthRad);

            local->rPos++;
            local->rSpeed = fields[3].toFloat()*3.2808399; // / cosAz;
            local->rDist  = fields[2].toFloat()/10.0; // * cosAz;
        }
    }
}

// ============================================================================
// Transponder polling state machine (timerAlt)
// ============================================================================

/**
 * @brief Periodically poll/configure the transponder.
 *
 * Simple state-machine that:
 *   - Asks for version, altitude, configuration, etc.
 *   - Optionally updates altitude based on barometer (if present).
 *
 * Called from timerAlt every ~150 ms.
 */
void MyTcpSocket::doTransponder()
{
    static int  state  = 0;

    if (!Transponderstat)
        return;

    switch (state) {
    case 0:
        readyWrite(const_cast<char *>("\x02" "v=1" "\x03"));
        break;
    case 1:
        readyWrite(const_cast<char *>("\x02" "z=?" "\x03"));
        break;
    case 2:
        readyWrite(const_cast<char *>("\x02" "a=?" "\x03"));
        break;
    case 3:
        readyWrite(const_cast<char *>("\x02" "c=?" "\x03"));
        break;
    case 4:
        readyWrite(const_cast<char *>("\x02" "s=?" "\x03"));
        break;
    case 5:
        readyWrite(const_cast<char *>("\x02" "i=?" "\x03"));
        break;
    case 6:
        readyWrite(const_cast<char *>("\x02" "y=?" "\x03"));
        break;
    case 7:
        readyWrite(const_cast<char *>("\x02" "d=?" "\x03"));
        break;
    case 8:
        // If transponder has internal barometer and we have a valid reading,
        // push current altitude to transponder to correct known bug.
        if (TransponderstatWithBarometer && m_pressure_raw > 1.0)
        {
            char x[64];
            snprintf(x, sizeof(x), "\x02" "a=%dM" "\x03",
                     static_cast<int>(m_preasure_alt * 0.3048));  // feet -> meters
            readyWrite(x);
        }
        break;
    default:
        break;
    }

    if (++state > 8) {
        state = 2;  // Loop among query states once initial setup is done
    }
}

// ============================================================================
// MQTT (X-Plane) message handling
// ============================================================================

/**
 * @brief Handle MQTT / X-Plane updates and map them to internal state.
 *
 * Topics currently supported (examples):
 *   - xplane/ax, xplane/ay, xplane/az       : accelerometer / gyro
 *   - xplane/roll, xplane/pitch, xplane/yaw : attitude
 *   - xplane/airspeed, xplane/climbRate     : air data
 *   - xplane/localPressure                  : barometric pressure
 *
 * @param ID     Topic string.
 * @param value  Parsed float payload.
 */
void MyTcpSocket::handleUpdate(const std::string &ID, const std::string &invalue)
{
    static bool first       = true;
    static int  statusCount = 0;

    const QString topic = QString::fromStdString(ID);
    qDebug() << "[MQTT]" << topic << "=" << invalue;

    if (ID == "xplane/engage") {
        // If we return acknowledged command...
        if(invalue == "Activated"){
            qDebug() << "Activated:" << invalue;
        }
        else if(invalue == "Dectivated"){
            qDebug() << "Dectivated:" << invalue;
        }
        ++statusCount;
    }
    else{
        float value = std::stof(invalue);

        if (ID == "xplane/topic") {
            qDebug() << "TOPIC:" << topic;
        } else if (ID == "xplane/ax") {
            m_has_MQTT_gyro = true;
            AccX = value;
            ++statusCount;
        } else if (ID == "xplane/ay") {
            m_has_MQTT_gyro = true;
            AccY = value;
            ++statusCount;
        } else if (ID == "xplane/az") {
            m_has_MQTT_gyro = true;
            AccZ = value;
            ++statusCount;
        } else if (ID == "xplane/rollRate") {
            m_has_MQTT_accel = true;
            AsX = value;
            ++statusCount;
        } else if (ID == "xplane/pitchRate") {
            m_has_MQTT_accel = true;
            AsY = value;
            ++statusCount;
        } else if (ID == "xplane/yawRate") {
            m_has_MQTT_accel = true;
            AsZ = value;
            ++statusCount;
        } else if (ID == "xplane/climbRate") {
            m_has_MQTT_vsi = true;
            ++statusCount;
        } else if (ID == "xplane/heading") {
            m_has_MQTT_heading = true;
            //m_heading = value;
            ++statusCount;
        } else if (ID == "xplane/airspeed") {
            m_has_MQTT_airspeed = true;
            m_speed = value;
            ++statusCount;
        } else if (ID == "xplane/localPressure") {
            m_has_MQTT_preassure = true;
            m_pressure_raw = value;
            ++statusCount;
        } else if (ID == "xplane/roll") {
            AngleX = value;
            ++statusCount;
        } else if (ID == "xplane/pitch") {
            AngleY = value;
            ++statusCount;
        } else if (ID == "xplane/yaw") {
            AngleZ = value;
            ++statusCount;
        }
    }
    // Mark MQTT as usable once we have a minimum number of key values
    if (statusCount >= 4 && first) {
        first = false;
        m_has_MQTT = true;
    }
}

// ============================================================================
// Low-level  helper
// ============================================================================
void MyTcpSocket::parseAirspeedLine(MyTcpSocket *thiz, const QString &line)
{
    bool ok1, ok2, ok3, ok4, ok5, ok6, ok7;

    // Remove whitespace and line endings
    QString clean = line.trimmed();

    // Split CSV
    QStringList parts = clean.split(QRegularExpression("[,\n]"),Qt::SkipEmptyParts);


    // Expect 5 fields or more...
    // the received data is buffered up so it might be mush more...
    if (parts.size() < 8)
        return;

    // Validate header
    if (parts[0] != "AIRSPEED")
        return;

    // Not used for now...,
    float pressure = parts[1].toFloat(&ok1);
    if (ok1 && !std::isnan(pressure)) {
        thiz->Airspeed_data.pressure    = pressure;
    }
    float temperature = parts[2].toFloat(&ok2);
    if (ok2 && !std::isnan(temperature)) {
        thiz->Airspeed_data.temperature    = temperature;
    }
    float dpPa = parts[3].toFloat(&ok3);
    if (ok3 && !std::isnan(dpPa)) {
        thiz->Airspeed_data.dpPa    = dpPa;
    }
    float offset = parts[4].toFloat(&ok4);
    if (ok4 && !std::isnan(offset)) {
        thiz->Airspeed_data.offset    = offset;
    }
    float corrected = parts[5].toFloat(&ok5);
    if (ok5 && !std::isnan(corrected)) {
        thiz->Airspeed_data.corrected    = corrected;
    }
//    float airspeed = parts[6].toFloat(&ok6);
    float airspeed = parts[3].toFloat(&ok6);
    if (ok6 && !std::isnan(airspeed)) {
        thiz->Airspeed_data.airspeed    = airspeed;
        thiz->m_airspeed = airspeed;
    }

    float angle = parts[7].toFloat(&ok7);
    if (ok5 && !std::isnan(angle)) {
        if(angle != 49999){ // If we got a valid angle...
            thiz->m_angle = angle;
        }
    }

    if (!(ok1 && ok2 && ok3 && ok4 && ok5 && ok6 && ok7))
        qDebug() << "one or more altitude sensor error";

//        return std::nullopt;
}

// ============================================================================
// Low-level  helper
// ============================================================================
void MyTcpSocket::parseAltimeterLine(MyTcpSocket *thiz, const QString &line)
{
    bool ok1, ok2, ok3, ok4, ok5;
    ok1=ok2=ok3=ok4=ok5=false;

    // Remove whitespace and line endings
    QString clean = line.trimmed();

    // Split CSV
    QStringList parts = clean.split(QRegularExpression("[,\n]"),Qt::SkipEmptyParts);

    // Expect 5 fields or more...
    // the received data is buffered up so it might be mush more...
    if (parts.size() < 6)
        return;

    // Validate header
    if (parts[0] != "ALTIMETER")
        return;

//    m_pressure_raw = parts[1].toFloat(&ok1);

    float pressure = parts[1].toFloat(&ok1);
    if (ok1 && !std::isnan(pressure)) {
        thiz->Altimeter_data.pressure    = pressure;
        //    m_pressure_raw = pressure;
    }

    float temperature = parts[2].toFloat(&ok2);
    if (ok2 && !std::isnan(temperature)) {
        thiz->Altimeter_data.temperature    = temperature;
    }

    float relative = parts[3].toFloat(&ok3);
    if (ok3 && !std::isnan(relative)) {
        thiz->Altimeter_data.relative    = relative;
    }

    float altitude = parts[4].toFloat(&ok4);
    if (ok4 && !std::isnan(altitude)) {
        thiz->Altimeter_data.altitude    = altitude;
         thiz->m_altitude = altitude;
    }

    float angle = parts[5].toFloat(&ok5);
    if (ok5 && !std::isnan(angle)) {
        if(angle != 49999){ // If we got a valid angle...
            thiz->m_angle = angle;
        }
    }

    if (!(ok1 && ok2 && ok3 && ok4 && ok5))
        qDebug() << "one or more altitude sensor error";
    //        return std::nullopt;
}

// ============================================================================
// Low-level write helper
// ============================================================================

/**
 * @brief Send a raw ASCII command to the transponder, if connected.
 *
 * @param data  Null-terminated C string (command).
 */
void MyTcpSocket::readyWrite(char *data)
{
    if (Transponderstat) {
        if(m_transponderClient != nullptr){
            if (m_transponderClient->state() == QAbstractSocket::ConnectedState) {
                m_transponderClient->write(data);
            }
            else{
                qDebug() << "Error NOT connectd!!!" << data;
            }
        }
        else{
#ifndef Q_OS_IOS
            TransponderSerPort->send(data);
#endif
        }
    }
}
