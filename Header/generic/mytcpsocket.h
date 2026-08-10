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
#include <QStandardPaths>
#include <QFile>
#include <QDir>

#include "serialport.h"

#define  TRANSPONDER_ONLY
#undef   USE_MQTT // Simulator...
#undef   USE_ANGLE

#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif

// Desktop platforms (non-Android, non-iOS) get Qt serial port support
#if not defined(Q_OS_ANDROID) && not defined(Q_OS_IOS)
#include <QSerialPort>
#include <QSerialPortInfo>
#endif

// --------------------------------------------------------------------------
// Platform-specific log/image directories
// --------------------------------------------------------------------------
#ifdef Q_OS_IOS
// iOS: user-visible Documents directory
#define LOG_DIR    QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/FlightInstrument"
#define IMAGES_DIR QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/FlightInstrument"
#elif defined(Q_OS_MAC)
// macOS: also use Documents
#define IMAGES_DIR QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/FlightInstrument"
#define LOG_DIR    QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/FlightInstrument"
#else
// Android: explicit external storage paths
#define IMAGES_DIR "/storage/emulated/0/DCIM/Camera"
#define LOG_DIR    "/storage/emulated/0/Documents"
#endif

// --------------------------------------------------------------------------
// File names (relative to LOG_DIR)
// --------------------------------------------------------------------------
#define RADIO          "/setup_radio_b.txt"
#define AIRPLANE       "/setup_ln_b.txt"
#define CONFIG         "/config_b.txt"
#define FLIGHTLOG      "/flightlog.txt"
#define TRANSPONDERLOG "/log.txt"


// -----------------------------------------------------------------------------
// UI selection / simulation flags
// -----------------------------------------------------------------------------
#ifdef TRANSPONDER_ONLY
 #ifdef Q_OS_ANDROID
  #ifdef LANDSCAPE
   #define SCREEN MainWindow_port_small
   #include "ui_mainwindow_port_small.h"
  #else
   #define SCREEN MainWindow_port_vertical
   #include "ui_mainwindow_port_vertical.h"
  #endif
 #else
    //#define SCREEN MainWindow_port_small
    //#include "ui_mainwindow_port_small.h"

    #define SCREEN MainWindow_port_vertical
    #include "ui_mainwindow_port_vertical.h"

#endif

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
                         void (*rety)(void *, bool use_imu) = nullptr);
    ~MyTcpSocket();

    /**
     * @brief Send a raw ASCII command to the transponder, if connected.
     *
     * @param data Null-terminated C string.
     */
    void readyWrite(const QByteArray &data);

    /**
     * @brief Initiate TCP/UDP connection if used (currently unused/stub).
     *
     * Left as API hook for potential future network functionality.
     */
    void doConnect();

    /// Callback to report IMU usage/availability to external C code.
    void (*ret_imu)(void *, bool use_imu) = nullptr;

    /// Callback invoked when serial data arrives from transponder.
    static void ret_altimeter(void *, const char *data, uint32_t size);

    void logdata(void *, QString file, QString datalog);

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
     * @brief Try to connect and initialize RADAR (Net or USB).
     *
     * Radar, with status dialogs along the way.
     */
    void connectedAltitude();
    void connectedAltitudeSerial();

    /**
     * @brief Android: periodically bump external display backlight to max.
     *
     * No-op on non-Android platforms.
     */
    void setbacklit();

    // Set the transponder to Serial mode og Callaham mode...
    void TransponderMode(bool mode);


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

    AltimeterData Altimeter_data = {0,0,0,0};

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

    /// Current serial port name (used by some legacy code).
    QString sport;

    /// Optional text log widget.
    QPlainTextEdit *text = nullptr;

    // Stansponder values...
    bool transponder_ping        = false;
    bool transponder_valid       = false;
    bool transponder_serial_mode = false;

    char transponder_command_s = '-';
    char transponder_command_r = '-';
    char transponder_command_i = '-';
    char transponder_command_c[10] = {'-'};
    char transponder_command_a[10] = {'-'};
    char transponder_command_z[20] = {'-'};
    char transponder_command_p = '-';

    int Transponder_altitude_mode = 0;


    // ------------------------------------------------------------------
    // Default USB serial numbers / IDs (SIM vs REAL)
    // ------------------------------------------------------------------
    QString _transponder_copy = "Transponder";
    QString _Altitude_copy    = "Altitude";

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
    bool Altitudestat = false;     ///< For convenience on macOS (no USB check yet).

    double m_preasure_QNH   = -10000; ///< Pressure-based altitude (feet), -10000 if invalid.
    bool   TransponderstatWithBarometer = false; ///< True if transponder has built-in barometer.

    quint16 VER = 0.0;       ///< IMU firmware version or similar (from "VER" field).
    double Temp = -100.0;      ///< IMU temperature [°C].

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

    float m_altitude = 0.0;


private:
    // ---------------------------------------------------------------------
    // Hardware communication backends
    // ---------------------------------------------------------------------
#ifndef Q_OS_IOS
    ComQt *TransponderSerPort = nullptr; ///< Serial port for transponder.
    ComQt *AltimeterPort      = nullptr; ///< Serial port for radar.
#endif

    QString m_transponder_address = "";
    QString m_altimeter_address = "";

    QTcpSocket *m_transponderClient = nullptr;
    QTcpSocket *m_altimeterClient = nullptr;

signals:
    /**
     * @brief Generic Qt signal for sending status / log messages to UI.
     */
    void sendMessage(const QString &message);

public slots:
    /**
     * @brief C-style RX callback from MyTcpSocket for transponder data.
     *
     * @param parent Pointer back to MainWindow instance.
     * @param data   Raw ASCII payload.
     * @param lenght Length of payload in bytes.
     */
    static void ret_transponder(void *parent, const char *data, uint32_t lenght);


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

private:
    // ---------------------------------------------------------------------
    // Internal timers
    // ---------------------------------------------------------------------
    QTimer *timerTRANS = nullptr; ///< Transponder polling timer.
    QTimer *timer      = nullptr; ///< Generic timer (used elsewhere).
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
