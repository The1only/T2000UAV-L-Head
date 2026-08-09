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
#include <QRegularExpression>

#include <QStringList>

#ifdef Q_OS_ANDROID
#include <QtCore/private/qandroidextras_p.h>
#include <QJniObject>
#include "sharedstorage.h"
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

#include <QVector>
#include <QString>
#include <QMap>
#include <unistd.h>   // usleep
#include "REG.h"

#ifndef USE_BT_IMU
    #define ComBt void
#endif

// Enable internal radar simulator (used when real radar is not present)
#undef SIMULATE_RADAR
typedef struct { double L, F, D; } LFD;

// External C interfaces (implemented elsewhere)
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
                         void (*rety)(void *, bool use_imu))
    : QObject(parent)
{
    this->ret_imu  = rety;  // C callback to notify IMU status
    this->text     = s;
    this->parent   = parent;

    // Now new app start...
    QString datalog = QDateTime::currentDateTime().toString()+": New Log:";

#ifdef Q_OS_ANDROID
    const bool success = SharedStorage::appendTextFile("LowEnergyScanner","log.txt",datalog);
    if (!success) {
        qWarning() << "Could not write transponder log";
    }
    else{
        qWarning() << QString(FLIGHTLOG) << " at " << "LowEnergyScanner";
    }

#else
    this->logdata(this, QString(TRANSPONDERLOG), datalog);
#endif

    // ---------------------------------------------------------------------
    // Serial / Bluetooth COM objects
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS

    // Altimeter serial port
    AltimeterPort = new ComQt(this);
    AltimeterPort->setParent(this);
    AltimeterPort->setRxCallback(ret_altimeter);         // callback from WIT C SDK

    // Transponder serial port
    TransponderSerPort = new ComQt(this);
    TransponderSerPort->setParent(this);
    TransponderSerPort->setRxCallback(ret_transponder);             // register C callback

    qDebug() << "Starting requester...";
    timerTRANS = new QTimer(this);
    timerTRANS->setSingleShot(false);
    connect(timerTRANS, SIGNAL(timeout()), SLOT(doTransponder()));
    timerTRANS->start(200);

    // Build a serial-number -> port map on macOS
    map = serialToPortMap(true);
    qDebug() << map;

#endif

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
    AltimeterPort->close();

#endif
    qDebug() << "Stopped socket...";
}

void MyTcpSocket::logdata(void* saved, QString logfile, QString datalog)
{
    Q_UNUSED(saved)
    Q_UNUSED(logfile)
    Q_UNUSED(datalog)

/*
    QString logDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(logDir);
    QString logPath = logDir + logfile;
    QFile file(logPath);
*/
//    this->text->appendPlainText(datalog);

/*
    datalog = datalog + "\n";

    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        const QByteArray data = datalog.toLocal8Bit();

        if (file.write(data) == -1) {
            QString logtxt = "Could not write log file: " + file.fileName()
            + " Error code: " + QString::number(static_cast<int>(file.error()))
                + " Error: " + file.errorString();
            qWarning() << logtxt;
        }
        file.close();
    }
    else
    {
        QString logtxt = "Could not open log file: " + file.fileName()
        +    " Error code: " + QString::number(static_cast<int>(file.error()))
            + " Error: " + file.errorString();
        qWarning() << logtxt;
    }
*/
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
    if (response.contains("T2000")) {
        portNum = "Transponder";
    }
    else if (response.contains("AIRSPEED")) {
        portNum = "Airspeed";
    }
    else if (response.contains("ALTIMETER")) {
        portNum = "Altitude";
    }
    else if (response.contains("RADAR")) {
        portNum = "Radar";
    }
    else if (response.contains("IMU")) {
        portNum = "Imu";
    }
    else if (response.contains("ANGLE")) {
        portNum = "Angle";
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
    Q_UNUSED(useSystemLocation)
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
    timerStart->stop();

    if (!Transponderstat){
        connected();
    }
    if (!Altitudestat){
        connectedAltitudeSerial();     // try to find & initialize Altimeter
    }    
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
void MyTcpSocket::TransponderMode(bool mode)
{
    static bool last_mode = true;

    if(mode != last_mode){
        if(mode){
            readyWrite(const_cast<char *>("\x02" "d=g" "\x03"));
            qDebug() << "d=g" ;
        }
        else{
            readyWrite(const_cast<char *>("\x02" "d=s" "\x03"));
            qDebug() << "d=s" ;
        }
        last_mode = mode;
    }
}


void MyTcpSocket::doTransponder()
{
    static int  state  = 0;

    if (!Transponderstat)
        return;

    switch (state) {
    case 0:
        readyWrite(const_cast<char *>("\x02" "v=1" "\x03"));
        readyWrite(const_cast<char *>("\x02" "d=g" "\x03"));
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
        if (transponder_serial_mode && m_altitude > 0.1)
        {
            char x[64];

            if(m_altitude > 160 || m_altitude < 140){
                qDebug() << m_altitude;
            }


            snprintf(x, sizeof(x), "\x02" "a=%dM" "\x03", static_cast<int>(m_altitude));  // feet -> meters
            readyWrite(x);
           // qDebug() << m_altitude;
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
// Low-level  helper
// 17:30:43.855 -> ALTIMETER,997.0312,24.2770,0.0898,135.8314,30031033                                                               17:30:44.052 -> ALTIMETER,997.0316,24.2846,0.0898,135.8314,30031033
// ============================================================================
void MyTcpSocket::parseAltimeterLine(MyTcpSocket *thiz, const QString &line)
{
    bool ok1, ok2, ok3, ok4, ok5;
    ok1=ok2=ok3=ok4=ok5=false;

    // Remove whitespace and line endings
    QString clean = line.trimmed();
    qDebug() << clean;

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
        thiz->Altimeter_data.altitude = altitude;
        thiz->m_altitude = altitude;
        if(altitude > 160 || altitude < 140){
            qDebug() << altitude;
        }
    }

    float angle = parts[5].toFloat(&ok5);
    if (ok5 && !std::isnan(angle)) {
    }

    if (!(ok1 && ok2 && ok3 && ok4 && ok5))
        qDebug() << "one or more altitude sensor error";
    //        return std::nullopt;
}

void MyTcpSocket::ret_transponder(void *parent, const char *data, uint32_t length) //const QByteArray &array)
{
    auto *local = static_cast<MyTcpSocket *>(parent);

    static bool bussy = false;
    static char buffer[30];
    static int pos = 0;

    if(bussy == false)
    {
        bussy = true;

        for(uint32_t i=0; i < length;i++)
        {
            if(data[i] == '*')
            {
                local->transponder_ping = true;
                QString datalog = QDateTime::currentDateTime().toString()+": "+"Ping received...";
                local->logdata(local, QString(TRANSPONDERLOG), datalog);
            }

            //        if(data[i] < 0x1F || pos >= (int)sizeof(buffer))
            if(data[i] == 0x03 || pos >= (int)sizeof(buffer))
            {
                if(pos >= 3)
                {
                    // Log all commands... This might be slow... will look at a timed write...
                    buffer[pos + 1] = '\0';
                    const QString datalog =
                        QDateTime::currentDateTime().toString(Qt::ISODate)
                        + ": "
                        + QString::fromLocal8Bit(buffer);

                    local->logdata(local, QString(TRANSPONDERLOG), datalog);
                  //  qDebug() << datalog;

                    switch (buffer[0])
                    {
                        case 's':
                        {
                            local->transponder_command_s = buffer[2];
                            local->transponder_valid = true;
                            break;
                        }
                        case 'd':
                        {
                            if(buffer[2] == 's'){
                                local->transponder_serial_mode = true;
                            }else{
                                local->transponder_serial_mode = false;
                            }
                            break;
                        }
                        case 'r':
                        {
                            local->transponder_command_r = buffer[2];
                            local->transponder_valid = true;
                            break;
                        }
                        case 'i':
                        {
                            local->transponder_command_i = buffer[2];
                            local->transponder_valid = true;
                            break;
                        }
                        case 'c':
                        {
                            strncpy(local->transponder_command_c,buffer,10);
                            local->transponder_valid = true;
                            break;
                        }
                        case 'a':
                        {
                            strncpy(local->transponder_command_a,buffer,10);
                            local->transponder_valid = true;
                            break;
                        }
                        case 'z':
                        {
                            strncpy(local->transponder_command_z,buffer,20);
                            local->transponder_valid = true;
                            break;
                        }
                        case 'p':
                        {
                            local->transponder_command_p = buffer[2];
                            local->transponder_valid = true;
                            break;
                        }
                    }
                }
                pos = 0;
            }
            else{
                if(data[i] != 0x02)  //>= 0x1F)
                {
                    buffer[pos++]=data[i];
                    buffer[pos]=0;
                }
                else{
                    pos = 0;
                }
            }
            QCoreApplication::processEvents();
        }
        bussy = false;
    }
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
