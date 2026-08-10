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
#include <QByteArray>
#include <QMutex>
#include <QMap>
#include <QVector>

#include <functional>

#include "serialport.h"


#define TRANSPONDER_ONLY

#undef USE_MQTT
#undef USE_ANGLE


#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif


#if !defined(Q_OS_ANDROID) && \
!defined(Q_OS_IOS)

#include <QSerialPort>
#include <QSerialPortInfo>

#endif


// ============================================================================
// Platform-specific directories
// ============================================================================

#ifdef Q_OS_IOS

#define LOG_DIR \
    QStandardPaths::writableLocation( \
        QStandardPaths::DocumentsLocation) + \
    "/FlightInstrument"

#define IMAGES_DIR \
    QStandardPaths::writableLocation( \
        QStandardPaths::DocumentsLocation) + \
    "/FlightInstrument"

#elif defined(Q_OS_MAC)

#define LOG_DIR \
QStandardPaths::writableLocation( \
    QStandardPaths::DocumentsLocation) + \
    "/FlightInstrument"

#define IMAGES_DIR \
    QStandardPaths::writableLocation( \
        QStandardPaths::DocumentsLocation) + \
    "/FlightInstrument"

#else

/*
 * Android shared files are handled through MediaStore.
 *
 * These legacy path definitions are retained because other source
 * files may still reference them. Direct QFile writes to these
 * locations should not be used on modern Android.
 */
#define IMAGES_DIR "/storage/emulated/0/DCIM/Camera"
#define LOG_DIR    "/storage/emulated/0/Documents"

#endif


// ============================================================================
// File names
// ============================================================================

#define RADIO          "/setup_radio_b.txt"
#define AIRPLANE       "/setup_ln_b.txt"
#define CONFIG         "/config_b.txt"
#define FLIGHTLOG      "/flightlog.txt"
#define TRANSPONDERLOG "/log.txt"


// ============================================================================
// UI selection
// ============================================================================

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

#define SCREEN MainWindow_port_vertical
#include "ui_mainwindow_port_vertical.h"

#endif

#define simGPS false

#endif


    // ============================================================================
    // Protocol framing
    // ============================================================================

    static constexpr char STX = 0x02;
static constexpr char ETX = 0x03;


#ifdef Q_OS_IOS
#define ComBt void
#define ComQt void
#endif


// ============================================================================
// Sensor data structures
// ============================================================================

struct AltimeterData
{
    float pressure;
    float temperature;
    float relative;
    float altitude;
};


struct AirspeedData
{
    float pressure;
    float temperature;
    float dpPa;
    float offset;
    float corrected;
    float airspeed;
};


// ============================================================================
// NoButtonMessageBox
// ============================================================================

class NoButtonMessageBox : public QDialog
{
    Q_OBJECT

public:

    explicit NoButtonMessageBox(
        const QString &message,
        QWidget *parent = nullptr)
        : QDialog(parent)
    {
        setWindowFlags(
            Qt::FramelessWindowHint |
            Qt::Dialog);

        setAttribute(
            Qt::WA_TranslucentBackground);

        setModal(true);

        auto *layout =
            new QVBoxLayout(this);

        m_label =
            new QLabel(
                message,
                this);

        m_label->setAlignment(
            Qt::AlignCenter);

        m_label->setStyleSheet(
            "QLabel { "
            "font-size: 18pt; "
            "color: white; "
            "background-color: #333; "
            "padding: 20px; "
            "border-radius: 12px; "
            "}");

        layout->addWidget(
            m_label);

        setLayout(layout);

        resize(300, 100);
    }


    void setText(
        const QString &text)
    {
        if (m_label)
        {
            m_label->setText(
                text);
        }
    }


private:

    QLabel *m_label = nullptr;
};


// ============================================================================
// PortEntry
// ============================================================================

struct PortEntry
{
    QString serial;
    QString portName;
    QString systemLocation;
    QString description;
    QString manufacturer;

    quint16 vendorId = 0;
    quint16 productId = 0;
};


// ============================================================================
// MyTcpSocket
// ============================================================================

/**
 * @brief Hardware communication subsystem.
 *
 * The historical class name is misleading: the class currently handles
 * serial, USB and TCP communication for multiple devices.
 *
 * Receive data is transferred to MainWindow using signals rather than
 * shared command buffers.
 */
class MyTcpSocket : public QObject
{
    Q_OBJECT


public:

    explicit MyTcpSocket(
        QObject *parent = nullptr,
        QPlainTextEdit *s = nullptr,
        void (*rety)(void *,
                     bool use_imu) = nullptr);

    ~MyTcpSocket();


    // =====================================================================
    // Communication
    // =====================================================================

    /**
     * @brief Send a complete transponder command.
     *
     * Cross-thread calls are automatically queued to this object's
     * owning thread.
     */
    void readyWrite(
        const QByteArray &data);


    /**
     * @brief Raw transponder serial callback.
     */
    static void ret_transponder(
        void *parent,
        const char *data,
        uint32_t length);


    /**
     * @brief Raw external-altimeter serial callback.
     */
    static void ret_altimeter(
        void *host,
        const char *data,
        uint32_t size);


    void doConnect();

    void connected();

    void transponderConnect();

    void connectedAltitude();

    void connectedAltitudeSerial();

    void setbacklit();

    void TransponderMode(bool mode);


    // =====================================================================
    // Data processing
    // =====================================================================

    void handleUpdate(
        const std::string &ID,
        const std::string &value);

    void parseAltimeterLine(
        MyTcpSocket *thiz,
        const QString &line);

    void logdata(
        void *,
        QString file,
        QString datalog);


    // =====================================================================
    // Serial-port discovery
    // =====================================================================

    static QVector<PortEntry>
    listSerialPortsDetailed();

    QMap<QString, QString>
    serialToPortMap(
        bool useSystemLocation = true);

    QString findPort(
        QString targetSerial);

    QMap<QString, QString> map;


    // =====================================================================
    // Miscellaneous existing public state
    // =====================================================================

    void (*ret_imu)(
        void *,
        bool use_imu) = nullptr;


    QString m_address =
        "239.255.0.1";

    quint16 MCAST_PORT =
        4210;


#if !defined(Q_OS_ANDROID) && \
    !defined(Q_OS_IOS)

        int com_setup(
            QSerialPort *com_port,
            QString sport);

    QSerialPort *port = nullptr;
    QSerialPort *lidar = nullptr;

    QList<QSerialPortInfo>
        serialport;

#endif


    QString sport;

    QPlainTextEdit *text = nullptr;


    // ---------------------------------------------------------------------
    // Device state
    // ---------------------------------------------------------------------

    bool Transponderstat = false;

    bool Altitudestat = false;

    int Transponder_altitude_mode = 0;


    // ---------------------------------------------------------------------
    // Device identifiers
    // ---------------------------------------------------------------------

    QString _transponder_copy =
        "Transponder";

    QString _Altitude_copy =
        "Altitude";


#ifdef Q_OS_ANDROID

    QJniObject *someJavaObject =
        nullptr;

    QJniObject *imuJavaObject =
        nullptr;

#else

    typedef struct Callbacks
    {
        void *classPtr;
        void (*callback)(void *);
    } Callbacks;

    Callbacks *callbacks =
        nullptr;


    void CallSwiftMemberFromC(
        void *classPtr,
        void (*callback)(void *))
    {
        callbacks->classPtr =
            classPtr;

        callbacks->callback =
            callback;

        std::function<void()> actualCallback =
            [&]()
        {
            callbacks->callback(
                callbacks->classPtr);
        };

        actualCallback();
    }

#endif


    // ---------------------------------------------------------------------
    // Existing sensor values
    // ---------------------------------------------------------------------

    QString imuData;

    double m_preasure_QNH =
        -10000;

    bool TransponderstatWithBarometer =
        false;

    quint16 VER = 0;

    double Temp = -100.0;

    double m_preasure_alt = 0.0;
    double m_preasure = 0.0;
    double m_pressure_raw = 0.0;

    double m_airspeed = 0.0;

    double m_speed = 0.0;
    double Donwn_Speed = 0.0;

    int Orient = 0;

    bool use_ins_only = false;


signals:

    // =====================================================================
    // General
    // =====================================================================

    void sendMessage(
        const QString &message);


    // =====================================================================
    // Transponder receive signals
    // =====================================================================

    void transponderActivity();

    void transponderPingReceived();

    void transponderModeReceived(
        char mode);

    void transponderAnnunciatorReceived(
        char value);

    void transponderIdentReceived(
        char value);

    void transponderCodeReceived(
        const QByteArray &command);

    void transponderAltitudeReceived(
        const QByteArray &command);

    void transponderTextReceived(
        const QByteArray &command);

    void transponderHardwareStatusReceived(
        char value);

    void transponderDataModeReceived(
        bool serialMode);


    // =====================================================================
    // External altimeter
    // =====================================================================

    void externalAltimeterReceived(
        float pressure,
        float temperature,
        float relative,
        float altitude);


public slots:

    /**
     * @brief Store altitude supplied by MainWindow.
     *
     * Executes in this object's thread when called through a queued
     * connection.
     */
    void setLocalAltitude(
        double altitudeMeters);


    /**
     * @brief Set selected altitude source.
     */
    void setTransponderAltitudeMode(
        int mode);


    void doTransponder();

    void doStart();


private:

    // =====================================================================
    // Receive parser
    // =====================================================================

    /**
     * @brief Protect per-instance transponder parser state.
     */
    QMutex m_transponderRxMutex;

    /**
     * @brief Current STX/ETX framed message being assembled.
     */
    QByteArray m_transponderRxBuffer;


    // =====================================================================
    // Internal altitude state
    // =====================================================================

    AltimeterData Altimeter_data =
        {0, 0, 0, 0};

    /**
     * @brief Active local altitude supplied to the transponder [m].
     *
     * Modified only in the MyTcpSocket thread.
     */
    double m_altitude = 0.0;

    /**
     * @brief Selected source mode, stored in this object's thread.
     */
    int m_transponderAltitudeMode = 0;

    /**
     * @brief Current transponder serial-altitude mode.
     */
    bool transponder_serial_mode = false;

    /**
     * @brief Previous requested TransponderMode() value.
     */
    bool m_lastTransponderMode = true;


    // =====================================================================
    // Communication backends
    // =====================================================================

#ifndef Q_OS_IOS

    ComQt *TransponderSerPort =
        nullptr;

    ComQt *AltimeterPort =
        nullptr;

#endif


    QString m_transponder_address =
        "";

    QString m_altimeter_address =
        "";

    QTcpSocket *m_transponderClient =
        nullptr;

    QTcpSocket *m_altimeterClient =
        nullptr;


    // =====================================================================
    // Timers
    // =====================================================================

    QTimer *timerTRANS =
        nullptr;

    QTimer *timer =
        nullptr;

    QTimer *java =
        nullptr;

    QTimer *timerStart =
        nullptr;


    QObject *parent =
        nullptr;


    int adapterFromUserSelection() const;

    int currentAdapterIndex =
        0;


    void reactOnSocketError(
        const QString &error);

    QString localName;
};

#endif // MYTCPSOCKET_H