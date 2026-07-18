#ifndef MYTCPSOCKET_H
#define MYTCPSOCKET_H

#include <QObject>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QAbstractSocket>
#include <QDebug>
#include <QTimer>
#include <QPlainTextEdit>
#include <QDialog>
#include <QVBoxLayout>
#include <QLabel>
#include <QList>
#include <QHostAddress>
#include <QNetworkInterface>

#include "ekfNavINS.h"
#include "rotation_matrix.h"
#include "serialport.h"
#include "mqttclient.h"
#include "tcpclient.h"
#include "ssdp.h"
#include "bleuart.h"

// Look for an external IMU over Bluetooth
#ifndef Q_OS_IOS
    #define USE_BT_IMU
#else
    #undef  USE_BT_IMU
#endif
//#undef  USE_BT_IMU


#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif

// Desktop platforms (non-Android, non-iOS) get Qt serial port support
#if not defined(Q_OS_ANDROID) && not defined(Q_OS_IOS)
#include <QSerialPort>
#include <QSerialPortInfo>
#endif

// -----------------------------------------------------------------------------
// UI selection / simulation flags
// -----------------------------------------------------------------------------
#ifdef Q_OS_ANDROID
// Android: main window layout
//#define SCREEN MainWindow_port_new
//#include "ui_mainwindow_port_new.h"

#define SCREEN MainWindow_port_small
#include "ui_mainwindow_port_small.h"

#include "lockhelper.h"
#define simGPS false

#elif defined(Q_OS_IOS)
// If IOS for apple...
#define SCREEN MainWindow_port_iPhone
#include "ui_mainwindow_port_iPhone.h"
#define simGPS false

#elif defined(Q_OS_MAC)
// If MAC or PC screen...
#include "ui_mainwindow_port_screen.h"
#define SCREEN MainWindow_port_screen
#define simGPS false

#else
// Desktop: new layout, GPS simulation enabled
#include "ui_mainwindow_port_new.h"
#define SCREEN MainWindow_port_new
#define simGPS false
#endif

// STX/ETX used for framing protocols (if needed elsewhere)
static constexpr char STX = 0x02;
static constexpr char ETX = 0x03;


#ifdef Q_OS_IOS
#define ComBt void
#define ComQt void
#endif

// ============================================================================
// NoButtonMessageBox
// ============================================================================
struct AltimeterData {
    float pressure;
    float temperature;
    float relative;
    float altitude;
};

struct AirspeedData {
    float pressure;
    float temperature;
    float dpPa;
    float offset;
    float corrected;
    float airspeed;
};

/**
 * @brief Small frameless dialog for transient status messages.
 *
 * A simple message box with no buttons, used to show progress / info like
 * "Looking for USB Transponder!" that auto-hides after a delay.
 */
class NoButtonMessageBox : public QDialog {
    Q_OBJECT
public:
    explicit NoButtonMessageBox(const QString &message, QWidget *parent = nullptr)
        : QDialog(parent)
    {
        setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
        setAttribute(Qt::WA_TranslucentBackground);
        setModal(true);

        auto *layout = new QVBoxLayout(this);
        m_label = new QLabel(message, this);
        m_label->setAlignment(Qt::AlignCenter);
        m_label->setStyleSheet(
            "QLabel { "
            "font-size: 18pt; "
            "color: white; "
            "background-color: #333; "
            "padding: 20px; "
            "border-radius: 12px; }"
            );
        layout->addWidget(m_label);
        setLayout(layout);
        resize(300, 100);
    }

    /**
     * @brief Update message text at runtime.
     *
     * @param text New text to show.
     */
    void setText(const QString &text)
    {
        if (m_label)
            m_label->setText(text);
    }

private:
    QLabel *m_label = nullptr;
};

// ============================================================================
// PortEntry
// ============================================================================

/**
 * @brief Convenience record for describing a serial port (used on macOS).
 */
struct PortEntry {
    QString serial;          ///< Device serial number (e.g. "4150323833373205").
    QString portName;        ///< User-facing port name (e.g. "COM5" or "cu.usbmodem1301").
    QString systemLocation;  ///< System path (e.g. "/dev/cu.usbmodem1301").
    QString description;     ///< Device description.
    QString manufacturer;    ///< Manufacturer string.
    quint16 vendorId = 0;    ///< USB vendor ID, if available.
    quint16 productId = 0;   ///< USB product ID, if available.
};

// ============================================================================
// MyTcpSocket
// ============================================================================

/**
 * @brief Handles communication with:
 *  - Transponder (USB serial)
 *  - Radar (USB serial or simulated)
 *  - IMU / INS (Bluetooth or USB serial)
 *  - X-Plane via MQTT (simulated sensor data)
 *
 * It also coordinates startup, periodically polls the transponder,
 * and exposes decoded IMU / radar values as public members.
 */
class MyTcpSocket : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Construct a MyTcpSocket.
     *
     * @param parent   QObject parent for ownership.
     * @param s        Optional text widget used for logging / debug output.
     * @param retx     C-style callback invoked when transponder data is received.
     * @param rety     C-style callback used to report IMU connection status.
     */
    explicit MyTcpSocket(QObject *parent = nullptr,
                         QPlainTextEdit *s = nullptr,
                         void (*retx)(void *, const char *, uint32_t) = nullptr,
                         void (*rety)(void *, bool use_imu) = nullptr);
    ~MyTcpSocket();

    void ssdpConfig();

    /**
     * @brief Send a raw ASCII command to the transponder, if connected.
     *
     * @param data Null-terminated C string.
     */
    void readyWrite(char *data);

    /**
     * @brief Initiate TCP/UDP connection if used (currently unused/stub).
     *
     * Left as API hook for potential future network functionality.
     */
    void doConnect();

    /// Callback to report IMU usage/availability to external C code.
    void (*ret_imu)(void *, bool use_imu) = nullptr;

    /// Callback invoked when serial data arrives from transponder.
    void (*ret_transponder)(void *, const char *data, uint32_t size) = nullptr;

    /// Callback invoked when serial data arrives from transponder.
    static void ret_airspeed(void *, const char *data, uint32_t size);

    /// Callback invoked when serial data arrives from transponder.
    static void ret_altimeter(void *, const char *data, uint32_t size);

    /**
     * @brief Try to connect and initialize the transponder.
     *
     * Starts transponder polling timer on success.
     */
    void connected();

    /**
     * @brief Try to connect and initialize the transponder over wlan.
     *
     * Starts transponder polling timer on success.
     */
    void transponderConnect();

    /**
     * @brief Try to connect and initialize IMU (BT and/or USB) and Radar.
     *
     * BT IMU → USB IMU → Radar, with status dialogs along the way.
     */
    void connectedIMU();

    /**
     * @brief Try to connect and initialize IMU (BT and/or USB) and Radar.
     *
     * BT IMU → USB IMU → Radar, with status dialogs along the way.
     */
    void connectedIMUWlan();

    /**
     * @brief Try to connect and initialize RADAR (Net or USB).
     *
     * Radar, with status dialogs along the way.
     */
    void connectedRadar();

    /**
     * @brief Try to connect and initialize RADAR (Net or USB).
     *
     * Radar, with status dialogs along the way.
     */
    void connectedRadarWlan();

    /**
     * @brief Try to connect and initialize RADAR (Net or USB).
     *
     * Radar, with status dialogs along the way.
     */
    void connectedAltitude();
    void connectedAltitudeSerial();

    /**
     * @brief Try to connect and initialize RADAR (Net or USB).
     *
     * Radar, with status dialogs along the way.
     */
    void connectedAirspeed();    
    void connectedAirSpeedSerial();

    /**
     * @brief Android: periodically bump external display backlight to max.
     *
     * No-op on non-Android platforms.
     */
    void setbacklit();

    /**
     * @brief Process an incoming MQTT message from the X-Plane bridge.
     *
     * Maps topics such as "xplane/roll", "xplane/ax", etc. into internal
     * IMU state variables and sets MQTT presence flags.
     *
     * @param ID    Topic name (e.g. "xplane/roll").
     * @param value Parsed float payload.
     */
    void handleUpdate(const std::string &ID, const std::string &value);


    void parseAltimeterLine(MyTcpSocket *thiz, const QString &line);
    void parseAirspeedLine( MyTcpSocket *thiz, const QString &line);

    AltimeterData Altimeter_data = {0,0,0,0};
    AirspeedData Airspeed_data = {0,0,0,0,0,-1};

//#ifdef Q_OS_MAC
    /**
     * @brief Enumerate serial ports with extra metadata (macOS).
     */
    static QVector<PortEntry> listSerialPortsDetailed();

    /**
     * @brief Build a map of serialNumber -> portPath.
     *
     * @param useSystemLocation If true, map to systemLocation; else to portName.
     */
    //static QMap<QString, QString> serialToPortMap(bool useSystemLocation = true);
    QMap<QString, QString> serialToPortMap(bool useSystemLocation = true);

    /**
     * @brief Find port path for a given USB serial number.
     *
     * @param targetSerial Serial number to search for.
     * @return Port path or empty string if not found.
     */
    QString findPort(QString targetSerial);

    /// Cached serialNumber -> port mapping for macOS.
    QMap<QString, QString> map;
//#endif

    QString m_address  = "239.255.0.1";
    quint16 MCAST_PORT = 4210;

#if not defined(Q_OS_ANDROID) && not defined(Q_OS_IOS)
    /**
     * @brief Configure a QSerialPort with given port name.
     *
     * @param com_port    Port instance.
     * @param sport       System port name (e.g. "COM3").
     * @return 0 on success, non-zero on error.
     */
    int com_setup(QSerialPort *com_port, QString sport);

    QSerialPort *port  = nullptr;  ///< Generic serial port (unused here; legacy).
    QSerialPort *lidar = nullptr;  ///< Optional lidar serial port (if used elsewhere).

    QList<QSerialPortInfo> serialport; ///< Cached list of available serial ports.
#endif
    SsdpDiscoverer *disc = nullptr;

    /// Current serial port name (used by some legacy code).
    QString sport;

    /// Optional text log widget.
    QPlainTextEdit *text = nullptr;

    // ------------------------------------------------------------------
    // Default USB serial numbers / IDs (SIM vs REAL)
    // ------------------------------------------------------------------
    QString _transponder_copy = "Transponder";
    QString _radar_copy       = "Radar";   ///< Default radar ID (SIM).
    QString _IMU_copy         = "Imu";         ///< REAL
    QString _AirSpeed_copy    = "Airspeed";
    QString _Altitude_copy    = "Altitude";


    // ------------------------------------------------------------------
    // Default USB serial numbers / IDs (SIM vs REAL)
    // ------------------------------------------------------------------
    QString _transponder_id = "9c:13:9e:f2:45:e8";   ///< Default transponder ID (SIM).
    //QString _transponder_id = "4150323833373205"; ///< REAL

    //QString _radar_id       = "415032383337320B";   ///< Default radar ID (SIM).
    // QString _radar_id       = "4150325537323317"; ///< REAL

    //QString _IMU_id          = "4150323833373009";   ///< Default IMU ID (SIM).
    //QString _IMU_id         = "FTHM2H8X";         ///< REAL
    //QString _IMU_id         = "29987";

    // ---------------------------------------------------------------------
    // MQTT state
    // ---------------------------------------------------------------------
    std::string SERVER_ADDRESS; ///< MQTT broker address (e.g. "tcp://localhost:1883").
    std::string CLIENT_ID;      ///< MQTT client ID (e.g. "transponder").
    MqttClient *mqtt = nullptr; ///< MQTT client instance.

    bool m_has_MQTT          = false; ///< True once enough MQTT values have been received.
    bool m_has_MQTT_gyro     = false;
    bool m_has_MQTT_accel    = false;
    bool m_has_MQTT_vsi      = false;
    bool m_has_MQTT_heading  = false;
    bool m_has_MQTT_airspeed = false;
    bool m_has_MQTT_preassure= false;

#ifdef Q_OS_ANDROID
    /// Helper Java object for transponder/USB operations.
    QJniObject *someJavaObject = nullptr;
    /// Helper Java object for IMU operations (if used).
    QJniObject *imuJavaObject  = nullptr;
#else
    // -----------------------------------------------------------------
    // iOS / desktop: generic callback bridge (e.g. for Swift interop).
    // -----------------------------------------------------------------

    typedef struct Callbacks
    {
        void *classPtr;
        void (*callback)(void *);
    } Callbacks;

    /// Global callback holder (must be created/released elsewhere).
    Callbacks *callbacks = nullptr;

    /**
     * @brief Example bridge to invoke Swift member functions from C code.
     *
     * Stores class pointer + function pointer in callbacks and executes it
     * via a lambda. Actual Swift bridging logic lives outside this file.
     */
    void CallSwiftMemberFromC(void *classPtr, void (*callback)(void *)) {
        callbacks->classPtr = classPtr;
        callbacks->callback = callback;

        std::function<void()> actualCallback = [&]() {
            callbacks->callback(callbacks->classPtr);
        };
        actualCallback();
    }
#endif

    // ---------------------------------------------------------------------
    // IMU / sensor data (decoded values)
    // ---------------------------------------------------------------------
    QString imuData;      ///< Last raw IMU ASCII payload.
//#ifdef Q_OS_MAC
//    bool Transponderstat = true;   ///< For convenience on macOS (no USB check yet).
//#else
 //   bool Transponderstat = true;  ///< True if transponder is connected and open.
    bool Transponderstat = false;  ///< True if transponder is connected and open.
//#endif
    bool Altitudestat = false;   ///< For convenience on macOS (no USB check yet).
    bool Airspeedstat = false;
    bool Radarstat    = false;       ///< True if radar device is connected.

    float rPos   = 0.0f;           ///< Raw radar "position" / bearing.
    float rSpeed = 0.0f;           ///< Radar radial speed along beam.
    float rDist  = 0.0f;           ///< Radar distance along beam.

    bool IMUconnected       = false; ///< True once IMU/INS is detected and streaming.
    bool m_external         = true;  ///< True if using external (HW) IMU vs simulation.
    bool m_imu_setup_done   = false; ///< True once IMU initialization has completed.

    double m_preasure_QNH   = -10000; ///< Pressure-based altitude (feet), -10000 if invalid.
    bool   TransponderstatWithBarometer = false; ///< True if transponder has built-in barometer.

    // IMU output variables (SI units where possible)
    QString FromID;
    double AccX = 0.0;      ///< Acceleration X [m/s^2].
    double AccY = 0.0;      ///< Acceleration Y [m/s^2].
    double AccZ = 0.0;      ///< Acceleration Z [m/s^2].
    double G    = Gfix;     ///< Local gravity constant.

    double AsX = 0.0;       ///< Angular speed X [deg/s or rad/s, depending on sensor].
    double AsY = 0.0;
    double AsZ = 0.0;

    double AngleX = 0.0;    ///< Roll angle [deg].
    double AngleY = 0.0;    ///< Pitch angle [deg].
    double AngleZ = 0.001;  ///< Yaw angle [deg].

    double HX = 0.0;        ///< Magnetometer X.
    double HY = 0.0;        ///< Magnetometer Y.
    double HZ = 0.0;        ///< Magnetometer Z.

    quint16 VER = 0.0;       ///< IMU firmware version or similar (from "VER" field).
    double Temp = -100.0;      ///< IMU temperature [°C].

    // GPS / altitude
    double m_angle         = 0.0; ///< GPS altitude [feet] (geoid-compensated).
    double m_altitude      = -100.0; ///< GPS altitude [feet] (geoid-compensated).
    double m_latitude      = 0.0; ///< GPS latitude [deg].
    double m_longitude     = 0.0; ///< GPS longitude [deg].

    // GPS velocity vector
    double m_gpsspeed   = 0.0; ///< GPS speed [m/s].
    double m_gpsbearing = 0.0; ///< GPS course [deg].
    double m_vel_N      = 0.0; ///< North velocity [m/s].
    double m_vel_E      = 0.0; ///< East velocity [m/s].
    double m_vel_D      = 0.0; ///< Down velocity [m/s].
    bool   m_vel_active = false;

    /// Barometric altitude [feet].
    double m_preasure_alt  = 0.0;
    /// Barometric pressure [hPa].
    double m_preasure      = 0.0;
    /// Raw barometric pressure [hPa] before offsets.
    double m_pressure_raw  = 0.0;
    double m_airspeed = 0.0;

    /// Ground speed [km/h].
    double m_speed  = 0.0;
    double Donwn_Speed  = 0.0; ///< Vertical speed (down) [m/s].

    int Orient = 0;            ///< Orientation mode / sensor orientation index.

    bool use_ins_only      = false;


private:
    QUdpSocket *m_multicastSender = nullptr;

    // ---------------------------------------------------------------------
    // Hardware communication backends
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS
    ComQt *TransponderSerPort = nullptr; ///< Serial port for transponder.
    ComQt *RadarSerPort       = nullptr; ///< Serial port for radar.
    ComQt *INSSerPort         = nullptr; ///< Serial port for IMU/INS.
    ComQt *AltimeterPort      = nullptr; ///< Serial port for radar.
    ComQt *AirSpeedPort       = nullptr; ///< Serial port for radar.
 #if defined(USE_BT_IMU)
    ComBt *bluetootPort       = nullptr; ///< Bluetooth port for IMU.
 #endif
#endif
    QString m_imu_address = "";
    QString m_radar_address = "";
    QString m_transponder_address = "";
    QString m_altimeter_address = "";
    QString m_airspeed_address = "";

    QTcpSocket *m_imuClient = nullptr;
    QTcpSocket *m_radarClient = nullptr;
    QTcpSocket *m_transponderClient = nullptr;
    QTcpSocket *m_altimeterClient = nullptr;
    QTcpSocket *m_airspeedClient = nullptr;

signals:
    /**
     * @brief Generic Qt signal for sending status / log messages to UI.
     */
    void sendMessage(const QString &message);

public slots:
    /**
     * @brief Periodic transponder polling / configuration state machine.
     *
     * Called from timerAlt every ~150 ms once transponder is connected.
     */
    void doTransponder();

    /**
     * @brief Startup state machine handler.
     *
     * Runs through IMU → Transponder → Radar setup steps.
     */
    void doStart();

    /**
     * @brief Static callback for IMU data (called from C driver).
     *
     * @param parent Pointer back to MyTcpSocket instance.
     * @param data   Null-terminated ASCII payload.
     * @param length Payload length in bytes.
     */
    //static void doIMU(void *parent, const char *data, uint32_t length);
    static void parseIMU(void *parent,uint32_t uiReg, uint16_t sReg[]);

    /**
     * @brief Static callback for Radar data (called from C driver).
     *
     * @param parent Pointer back to MyTcpSocket instance.
     * @param data   Null-terminated ASCII payload.
     * @param length Payload length in bytes.
     */
    static void doRadar(void *parent, const char *data, uint32_t length);

    // void SensorUartSend(uint8_t *p_data, uint32_t uiSize);  // legacy, unused

private:
    // ---------------------------------------------------------------------
    // Internal timers
    // ---------------------------------------------------------------------
    QTimer *timerTRANS = nullptr; ///< Transponder polling timer.
    QTimer *timer      = nullptr; ///< Generic timer (used elsewhere).
    QTimer *timerIMU   = nullptr; ///< IMU-related timer (if used).
    QTimer *java       = nullptr; ///< Android Java helper timer (if used).
    QTimer *timerStart = nullptr; ///< Startup state-machine timer.

    QObject *parent = nullptr;    ///< Cached parent pointer.

    int adapterFromUserSelection() const; ///< Map UI adapter selection to internal index.
    int currentAdapterIndex = 0;          ///< Currently selected adapter index.

    /**
     * @brief React to socket/connection errors (currently unused).
     *
     * @param error Error description.
     */
    void reactOnSocketError(const QString &error);

    QString localName; ///< Local adapter name / identifier (if used).
};

#endif // MYTCPSOCKET_H
