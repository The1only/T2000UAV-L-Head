/**
 * @file mytcpsocket.cpp
 * @brief Implementation of the communication subsystem.
 *
 * This file implements the communication layer used by the application for
 * transponder and external-altimeter access.
 *
 * Despite the historical class name, MyTcpSocket is not limited to TCP.
 * It currently supports:
 * - Transponder communication over serial/USB.
 * - Optional transponder communication over TCP.
 * - External altimeter communication over serial/USB.
 * - External altimeter communication over TCP.
 * - Serial-device discovery and identification.
 * - Periodic transponder polling.
 * - Parsing of STX/ETX framed transponder messages.
 * - Parsing of external-altimeter CSV messages.
 * - Thread-safe transfer of received values using Qt signals.
 * - Android-specific Java/JNI helper functionality.
 *
 * Incoming transponder data is parsed into complete protocol messages and
 * emitted as Qt signals. MainWindow therefore does not directly access the
 * transponder receive buffer.
 *
 * The intended receive path is:
 *
 * @code
 * Serial callback
 *      |
 *      v
 * ret_transponder()
 *      |
 *      +--> assemble STX/ETX frame
 *      |
 *      +--> emit signal carrying copied data
 *                |
 *                v
 *          Qt queued connection
 *                |
 *                v
 *            MainWindow
 * @endcode
 *
 * External-altimeter data follows a similar model. Raw callback data is
 * transferred into the MyTcpSocket thread before parsing, after which a
 * complete measurement is emitted to MainWindow.
 */

#define UDP

#include "mytcpsocket.h"

#ifdef Q_OS_ANDROID
#include <QtCore/private/qandroidextras_p.h>
#include <QJniObject>
#include "sharedstorage.h"
#endif

#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif

#include <QTime>
#include <QTimer>
#include <QThread>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QRegularExpression>
#include <QStringList>
#include <QList>
#include <QDebug>
#include <QVector>
#include <QString>
#include <QMap>
#include <QMetaObject>
#include <QMutexLocker>
#include <QDateTime>

#ifndef Q_OS_IOS
#include <QSerialPortInfo>
#endif

#include <unistd.h>

#include <cmath>
#include <cstdio>

#include "REG.h"


#ifndef USE_BT_IMU
#define ComBt void
#endif


/*
 * Disable the internal radar simulator for this build.
 */
#undef SIMULATE_RADAR


/**
 * @brief Small legacy structure used for L/F/D values.
 *
 * The exact interpretation of these values belongs to functionality outside
 * the code shown in this file.
 */
typedef struct
{
    double L;
    double F;
    double D;
} LFD;


/*
 * External sensor/helper functions implemented elsewhere.
 */
extern bool AutoScanSensor();
extern void AutoSetBaud(int);


// ============================================================================
// Serial probe state
// ============================================================================

/**
 * @brief File-local state used while identifying serial devices.
 *
 * Serial-device probing installs ret_test() as a temporary receive callback.
 * Because that callback may execute in a serial-worker thread while the
 * probing function is running elsewhere, access to g_portNum is protected by
 * g_portProbeMutex.
 *
 * The anonymous namespace keeps these variables private to this translation
 * unit.
 */
namespace
{

/*
 * Protect the serial-device identification result.
 */
QMutex g_portProbeMutex;

/*
 * Logical name assigned to the most recently identified serial device.
 *
 * Examples:
 * - "Transponder"
 * - "Altitude"
 * - "Radar"
 * - "Imu"
 */
QString g_portNum;

} // namespace


// ============================================================================
// Constructor / Destructor
// ============================================================================

/**
 * @brief Construct the communication subsystem.
 *
 * Initialization performed here includes:
 * - Store callback and UI/debug references.
 * - Create a new communication-session log entry.
 * - Create the external-altimeter serial interface.
 * - Create the transponder serial interface.
 * - Install their receive callbacks.
 * - Start periodic transponder polling.
 * - Probe available serial devices.
 * - Build the logical-device-to-port map.
 * - Start a short deferred-start timer.
 *
 * The actual device connection attempt is deferred so the constructor can
 * complete before connected() and connectedAltitudeSerial() are called.
 *
 * @param parent QObject parent used for Qt ownership.
 * @param s Optional diagnostic QPlainTextEdit.
 * @param rety Optional C-style callback used for reporting IMU availability.
 */
MyTcpSocket::MyTcpSocket(
    QObject *parent,
    QPlainTextEdit *s,
    void (*rety)(void *,
                 bool use_imu))
    : QObject(parent)
{
    /*
     * Store optional callback and external object references.
     */
    ret_imu =
        rety;

    text =
        s;

    this->parent =
        parent;


    /*
     * Mark the beginning of a new communication/logging session.
     */
    const QString datalog =
        QDateTime::currentDateTime().toString() +
        ": New Log:";


#ifdef Q_OS_ANDROID

    /*
     * Android:
     * Write to public/shared storage through SharedStorage / MediaStore.
     */
    const bool success =
        SharedStorage::appendTextFile(
            "LowEnergyScanner",
            "log.txt",
            datalog);

    if (!success)
    {
        qWarning()
        << "Could not write transponder log";
    }

#else

    /*
     * Other platforms use the generic logdata() helper.
     */
    logdata(
        this,
        QString(TRANSPONDERLOG),
        datalog);

#endif


#ifndef Q_OS_IOS

    // ---------------------------------------------------------------------
    // External altimeter serial interface
    // ---------------------------------------------------------------------

    /*
     * Create the serial abstraction used by the external altimeter.
     */
    AltimeterPort =
        new ComQt(this);

    AltimeterPort->setParent(this);

    /*
     * Raw altimeter bytes are delivered to ret_altimeter().
     */
    AltimeterPort->setRxCallback(
        ret_altimeter);


    // ---------------------------------------------------------------------
    // Transponder serial interface
    // ---------------------------------------------------------------------

    /*
     * Create the serial abstraction used by the transponder.
     */
    TransponderSerPort =
        new ComQt(this);

    TransponderSerPort->setParent(this);

    /*
     * Raw transponder bytes are delivered to ret_transponder().
     */
    TransponderSerPort->setRxCallback(
        ret_transponder);


    // ---------------------------------------------------------------------
    // Periodic transponder polling
    // ---------------------------------------------------------------------

    /*
     * doTransponder() periodically requests current transponder state and
     * optionally sends external altitude data.
     */
    timerTRANS =
        new QTimer(this);

    connect(
        timerTRANS,
        &QTimer::timeout,
        this,
        &MyTcpSocket::doTransponder);

    /*
     * Poll one state every 200 ms.
     */
    timerTRANS->start(200);


    // ---------------------------------------------------------------------
    // Serial-port discovery
    // ---------------------------------------------------------------------

    /*
     * Probe available serial ports and build a map such as:
     *
     * "Transponder" -> device path/index
     * "Altitude"    -> device path/index
     */
    map =
        serialToPortMap(true);

    qDebug()
        << "Device map:"
        << map;

#endif


    // ---------------------------------------------------------------------
    // Deferred startup
    // ---------------------------------------------------------------------

    /*
     * Device opening is deferred by 500 ms so initialization and discovery
     * can finish before connection attempts begin.
     */
    timerStart =
        new QTimer(this);

    timerStart->setSingleShot(true);

    connect(
        timerStart,
        &QTimer::timeout,
        this,
        &MyTcpSocket::doStart);

    timerStart->start(500);
}


/**
 * @brief Destroy the communication subsystem.
 *
 * Closes active serial interfaces on platforms where they are available.
 * Qt-owned child objects are subsequently destroyed through QObject
 * ownership.
 */
MyTcpSocket::~MyTcpSocket()
{
#ifndef Q_OS_IOS

    if (TransponderSerPort)
    {
        TransponderSerPort->close();
    }

    if (AltimeterPort)
    {
        AltimeterPort->close();
    }

#endif

    qDebug()
        << "Communication subsystem stopped";
}


// ============================================================================
// Logging
// ============================================================================

/**
 * @brief Generic communication logging helper.
 *
 * The logging implementation is currently disabled, but the function is kept
 * as a common call point for existing code.
 *
 * @param saved Historical/context pointer, currently unused.
 * @param logfile Requested target logfile, currently unused.
 * @param datalog Text to be written, currently unused.
 */
void MyTcpSocket::logdata(
    void *saved,
    QString logfile,
    QString datalog)
{
    Q_UNUSED(saved)
    Q_UNUSED(logfile)
    Q_UNUSED(datalog)
}


// ============================================================================
// Serial enumeration
// ============================================================================

/**
 * @brief Enumerate available serial ports and collect detailed metadata.
 *
 * For each available QSerialPortInfo entry, the function stores:
 * - Serial number.
 * - Port name.
 * - System location.
 * - Description.
 * - Manufacturer.
 * - USB vendor ID.
 * - USB product ID.
 *
 * On iOS the serial-port enumeration block is excluded.
 *
 * @return Vector containing one PortEntry for each available serial port.
 */
QVector<PortEntry>
MyTcpSocket::listSerialPortsDetailed()
{
    QVector<PortEntry> out;

#ifndef Q_OS_IOS

    const auto ports =
        QSerialPortInfo::availablePorts();

    out.reserve(
        ports.size());

    for (const QSerialPortInfo &p :
         ports)
    {
        PortEntry e;

        e.serial =
            p.serialNumber();

        e.portName =
            p.portName();

        e.systemLocation =
            p.systemLocation();

        e.description =
            p.description();

        e.manufacturer =
            p.manufacturer();

        e.vendorId =
            p.hasVendorIdentifier()
                ? p.vendorIdentifier()
                : 0;

        e.productId =
            p.hasProductIdentifier()
                ? p.productIdentifier()
                : 0;

        out.push_back(e);
    }

#endif

    return out;
}


/**
 * @brief Receive callback used while probing unidentified serial devices.
 *
 * During startup, each available serial port is opened temporarily and sent
 * an identification request. The returned text is inspected for known device
 * identifiers.
 *
 * Recognized responses:
 * - "T2000"     -> Transponder
 * - "AIRSPEED"  -> Airspeed
 * - "ALTIMETER" -> Altitude
 * - "RADAR"     -> Radar
 * - "IMU"       -> Imu
 * - "ANGLE"     -> Angle
 *
 * If a known device is detected, g_portNum is updated while protected by
 * g_portProbeMutex.
 *
 * @param userData Callback context, currently unused.
 * @param data Raw received bytes.
 * @param size Number of valid bytes in @p data.
 */
static void ret_test(
    void *userData,
    const char *data,
    uint32_t size)
{
    Q_UNUSED(userData)

    /*
     * Preserve the exact number of bytes supplied by the serial callback.
     */
    const QByteArray bytes(
        data,
        static_cast<int>(size));

    /*
     * Convert response text and remove surrounding whitespace/newlines.
     */
    const QString response =
        QString::fromUtf8(bytes)
            .trimmed();


    QString detectedDevice;


    /*
     * Match known identification strings.
     */
    if (response.contains("T2000"))
    {
        detectedDevice =
            "Transponder";
    }
    else if (response.contains("AIRSPEED"))
    {
        detectedDevice =
            "Airspeed";
    }
    else if (response.contains("ALTIMETER"))
    {
        detectedDevice =
            "Altitude";
    }
    else if (response.contains("RADAR"))
    {
        detectedDevice =
            "Radar";
    }
    else if (response.contains("IMU"))
    {
        detectedDevice =
            "Imu";
    }
    else if (response.contains("ANGLE"))
    {
        detectedDevice =
            "Angle";
    }


    /*
     * Store a valid identification result.
     */
    if (!detectedDevice.isEmpty())
    {
        QMutexLocker<QMutex> locker(
            &g_portProbeMutex);

        g_portNum =
            detectedDevice;

        qDebug()
            << "Found device:"
            << g_portNum;
    }
}


/**
 * @brief Probe available serial ports and build a logical device map.
 *
 * Every available port is temporarily opened at 9600 baud and sent the
 * identification request:
 *
 * @code
 * STX z=? ETX
 * @endcode
 *
 * The request is repeated four times with 500 ms delays. ret_test() examines
 * returned data and stores the detected logical device type.
 *
 * Android:
 * - Devices are opened using their enumeration index.
 *
 * Other supported platforms:
 * - Devices are opened using systemLocation.
 *
 * The resulting map is used later by findPort().
 *
 * Example:
 *
 * @code
 * "Transponder" -> "/dev/cu.usbserial..."
 * "Altitude"    -> "/dev/cu.usbmodem..."
 * @endcode
 *
 * @param useSystemLocation
 *        true to store QSerialPortInfo::systemLocation().
 *        false to store QSerialPortInfo::portName().
 *
 * @return Map from logical device name to serial-port identifier.
 */
QMap<QString, QString>
MyTcpSocket::serialToPortMap(
    bool useSystemLocation)
{
    QMap<QString, QString> result;

    /*
     * Delay used both before discovery and between probe requests.
     */
    const useconds_t waitTime =
        static_cast<useconds_t>(
            500 * 1000u);

    /*
     * Android uses the enumeration index as the serial-device identifier.
     */
    int i = 0;


#ifndef Q_OS_IOS

    /*
     * Give USB/serial devices time to initialize after startup.
     */
    usleep(waitTime);


    for (const auto &e :
         listSerialPortsDetailed())
    {
        /*
         * Clear the previous detection result before probing the next port.
         */
        {
            QMutexLocker<QMutex> locker(
                &g_portProbeMutex);

            g_portNum.clear();
        }


        /*
         * Include millisecond timestamps in Qt diagnostic output.
         */
        qSetMessagePattern(
            "%{time HH:mm:ss.zzz} "
            "%{type}: %{message}");


        qDebug()
            << "SerialPort:"
            << e.systemLocation
            << e.portName
            << e.serial
            << e.productId
            << e.vendorId;


        /*
         * Ignore entries without a usable port name.
         */
        if (!e.portName.isEmpty())
        {
            /*
             * Create a temporary serial interface for this probe.
             */
            ComQt *TestSerPort =
                new ComQt();

            TestSerPort->setParent(
                this);

            TestSerPort->setRxCallback(
                ret_test);


#ifdef Q_OS_ANDROID

            /*
             * Android ComQt uses the serial-device enumeration index.
             */
            const bool opened =
                TestSerPort->open(
                    QString::number(i),
                    QSerialPort::Baud9600);

#else

            /*
             * Desktop/macOS uses the system serial-device path.
             */
            const bool opened =
                TestSerPort->open(
                    e.systemLocation,
                    QSerialPort::Baud9600);

#endif


            /*
             * Repeatedly request device identification.
             */
            if (opened)
            {
                for (int attempt = 0;
                     attempt < 4;
                     ++attempt)
                {
                    TestSerPort->send(
                        "\x02"
                        "z=?"
                        "\x03");

                    usleep(waitTime);
                }
            }


            /*
             * The temporary probe interface is no longer required.
             */
            TestSerPort->close();

            delete TestSerPort;


            /*
             * Copy the result while holding the probe mutex.
             */
            QString detectedDevice;

            {
                QMutexLocker<QMutex> locker(
                    &g_portProbeMutex);

                detectedDevice =
                    g_portNum;
            }


#ifdef Q_OS_ANDROID

            /*
             * On Android, store the device enumeration index.
             */
            if (!detectedDevice.isEmpty())
            {
                result.insert(
                    detectedDevice,
                    QString::number(i));
            }

            ++i;

#else

            /*
             * Store either the system path or the shorter port name.
             */
            if (!detectedDevice.isEmpty())
            {
                result.insert(
                    detectedDevice,
                    useSystemLocation
                        ? e.systemLocation
                        : e.portName);
            }

#endif
        }
    }

#endif


    qDebug()
        << result;

    return result;
}


/**
 * @brief Look up the serial-port identifier associated with a device.
 *
 * @param targetSerial Logical device name stored in @c map, such as
 *                     "Transponder" or "Altitude".
 *
 * @return Associated serial-port identifier, or an empty QString when no
 *         matching device has been discovered.
 */
QString MyTcpSocket::findPort(
    QString targetSerial)
{
    if (map.contains(targetSerial))
    {
        return map.value(
            targetSerial);
    }

    qWarning()
        << "Device"
        << targetSerial
        << "not found";

    return {};
}


// ============================================================================
// Deferred startup
// ============================================================================

/**
 * @brief Perform deferred hardware connection attempts.
 *
 * This function is called once by timerStart after the constructor has
 * completed.
 *
 * Current sequence:
 * - Connect the transponder if it is not already connected.
 * - Connect the serial external altimeter if it is not already connected.
 */
void MyTcpSocket::doStart()
{
    if (!Transponderstat)
    {
        connected();
    }

    if (!Altitudestat)
    {
        connectedAltitudeSerial();
    }
}


// ============================================================================
// Thread-owned altitude configuration
// ============================================================================

/**
 * @brief Store locally supplied altitude inside the communication subsystem.
 *
 * MainWindow sends altitude through a queued Qt signal rather than directly
 * modifying @c m_altitude. The value is therefore normally written in this
 * object's owning thread.
 *
 * This altitude may later be sent to the transponder by doTransponder().
 *
 * @param altitudeMeters Altitude in meters.
 */
void MyTcpSocket::setLocalAltitude(
    double altitudeMeters)
{
    m_altitude =
        altitudeMeters;
}


/**
 * @brief Store the altitude-source selection.
 *
 * The new source mode is stored internally and mirrored into the legacy
 * Transponder_altitude_mode member.
 *
 * Current mode interpretation:
 * - 0 : transponder altitude.
 * - 1 : external altimeter.
 * - 2 : automatic selection.
 * - 3 : internal pressure sensor.
 *
 * @param mode Altitude-source mode.
 */
void MyTcpSocket::setTransponderAltitudeMode(
    int mode)
{
    m_transponderAltitudeMode =
        mode;

    /*
     * Retain public legacy member for code elsewhere that may still inspect it.
     *
     * Both assignments happen in this object's thread.
     */
    Transponder_altitude_mode =
        mode;
}


// ============================================================================
// Android backlight helper
// ============================================================================

/**
 * @brief Periodically request full backlight on an Android helper device.
 *
 * Android-only functionality.
 *
 * The Java helper class:
 *
 * @code
 * com.hoho.android.usbserial.driver.TestClassTerje
 * @endcode
 *
 * is instantiated lazily using the Android application context.
 *
 * Every time the internal counter exceeds 40 calls, Java method change(255)
 * is invoked.
 *
 * On non-Android platforms this function performs no action.
 */
void MyTcpSocket::setbacklit()
{
#ifdef Q_OS_ANDROID

    /*
     * Initial value forces the first call to update the backlight.
     */
    static int disp = 999;

    /*
     * Lazily construct the Java helper.
     */
    if (someJavaObject == nullptr)
    {
        QJniEnvironment env;

        auto context =
            QJniObject(
                QNativeInterface::
                QAndroidApplication::
                context());


        if (QJniObject::isClassAvailable(
                "com/hoho/android/usbserial/driver/TestClassTerje"))
        {
            someJavaObject =
                new QJniObject(
                    "com/hoho/android/usbserial/driver/TestClassTerje",
                    "(Landroid/content/Context;)V",
                    context.object());
        }
    }


    /*
     * Refresh backlight approximately every 41 invocations.
     */
    if (++disp > 40 &&
        someJavaObject != nullptr)
    {
        disp = 0;

        const int result =
            someJavaObject->callMethod<jint>(
                "change",
                "(I)I",
                255);

        qDebug()
            << "Display Backlit set to:"
            << result;
    }

#endif
}


// ============================================================================
// Device connection
// ============================================================================

/**
 * @brief Connect the transponder over the discovered serial interface.
 *
 * findPort() resolves the logical transponder identifier to the actual serial
 * interface discovered during startup.
 *
 * The transponder serial link uses 9600 baud.
 *
 * On successful open, Transponderstat is set true.
 */
void MyTcpSocket::connected()
{
#ifndef Q_OS_IOS

    const QString transponderName =
        findPort(
            _transponder_copy);

    if (!transponderName.isEmpty() &&
        TransponderSerPort->open(
            transponderName,
            QSerialPort::Baud9600))
    {
        Transponderstat =
            true;
    }

#endif
}


/**
 * @brief Connect the external altimeter over TCP.
 *
 * If m_altimeter_address contains a valid host address:
 * - Any previous TCP client is deleted.
 * - A new QTcpSocket is created.
 * - connected/disconnected signals update Altitudestat.
 * - readyRead() forwards received bytes to parseAltimeterLine().
 * - TCP connection is initiated on port 23.
 */
void MyTcpSocket::connectedAltitude()
{
    /*
     * No TCP endpoint has been configured.
     */
    if (m_altimeter_address.isEmpty())
        return;


    /*
     * Replace any previous altimeter TCP socket.
     */
    if (m_altimeterClient)
    {
        delete m_altimeterClient;
    }


    m_altimeterClient =
        new QTcpSocket(this);


    /*
     * Track successful TCP connection.
     */
    connect(
        m_altimeterClient,
        &QTcpSocket::connected,
        this,
        [this]()
        {
            qDebug()
            << "Altimeter TCP connected";

            Altitudestat =
                true;
        });


    /*
     * Track TCP disconnection.
     */
    connect(
        m_altimeterClient,
        &QTcpSocket::disconnected,
        this,
        [this]()
        {
            qDebug()
            << "Altimeter TCP disconnected";

            Altitudestat =
                false;
        });


    /*
     * Parse all currently available altimeter data whenever the TCP socket
     * reports readyRead().
     */
    connect(
        m_altimeterClient,
        &QTcpSocket::readyRead,
        this,
        [this]()
        {
            const QByteArray data =
                m_altimeterClient->readAll();

            if (!data.isEmpty())
            {
                parseAltimeterLine(
                    this,
                    QString::fromLatin1(data));
            }
        });


    /*
     * Port 23 is used by the current network-altimeter implementation.
     */
    m_altimeterClient->connectToHost(
        QHostAddress(
            m_altimeter_address),
        23);
}


/**
 * @brief Connect the external altimeter over serial/USB.
 *
 * The logical "Altitude" device mapping is resolved using findPort().
 * The serial altimeter operates at 115200 baud.
 *
 * On success Altitudestat is set true.
 */
void MyTcpSocket::connectedAltitudeSerial()
{
#ifndef Q_OS_IOS

    const QString portName =
        findPort(
            _Altitude_copy);

    qDebug()
        << "Looking for Altimeter Port:"
        << portName;


    if (!portName.isEmpty() &&
        AltimeterPort->open(
            portName,
            QSerialPort::Baud115200))
    {
        Altitudestat =
            true;
    }

#endif
}


// ============================================================================
// External-altimeter callback
// ============================================================================

/**
 * @brief Raw serial receive callback for the external altimeter.
 *
 * The serial backend may invoke this callback outside the MyTcpSocket thread.
 * The incoming bytes are therefore copied into a QString and transferred to
 * the MyTcpSocket event queue using QMetaObject::invokeMethod().
 *
 * parseAltimeterLine() then runs in the thread that owns the MyTcpSocket
 * object.
 *
 * @param host Callback context. Expected to point to MyTcpSocket.
 * @param data Raw received bytes.
 * @param size Number of valid bytes.
 */
void MyTcpSocket::ret_altimeter(
    void *host,
    const char *data,
    uint32_t size)
{
    auto *local =
        static_cast<MyTcpSocket *>(host);

    /*
     * Reject invalid callback arguments.
     */
    if (!local ||
        !data ||
        size == 0)
    {
        return;
    }


    /*
     * Copy the callback buffer because the original memory may no longer be
     * valid after this callback returns.
     */
    const QString text =
        QString::fromLatin1(
            data,
            static_cast<qsizetype>(
                size));


    /*
     * Ensure parser runs in the MyTcpSocket thread.
     */
    QMetaObject::invokeMethod(
        local,
        [local, text]()
        {
            local->parseAltimeterLine(
                local,
                text);
        },
        Qt::QueuedConnection);
}


// ============================================================================
// Transponder altitude-data mode
// ============================================================================

/**
 * @brief Change the transponder altitude-data operating mode.
 *
 * This function may be called from another thread. If so, the operation is
 * queued back to the MyTcpSocket thread before accessing transponder state.
 *
 * Commands:
 * - mode == true  -> @c d=g
 * - mode == false -> @c d=s
 *
 * The most recently requested mode is stored in m_lastTransponderMode so
 * duplicate commands are not repeatedly transmitted.
 *
 * @param mode Requested altitude-data mode.
 */
void MyTcpSocket::TransponderMode(
    bool mode)
{
    /*
     * Ensure the state and command are handled in this object's thread.
     */
    if (QThread::currentThread() !=
        thread())
    {
        QMetaObject::invokeMethod(
            this,
            [this, mode]()
            {
                TransponderMode(mode);
            },
            Qt::QueuedConnection);

        return;
    }


    /*
     * No command is required when the requested state has not changed.
     */
    if (mode ==
        m_lastTransponderMode)
    {
        return;
    }


    /*
     * Send the corresponding transponder protocol command.
     */
    if (mode)
    {
        readyWrite(QByteArray("\x02" "d=g" "\x03",5));
        qDebug() << "d=g";
    }
    else
    {
        readyWrite(QByteArray("\x02" "d=s" "\x03",5));
        qDebug() << "d=s";
    }


    /*
     * Remember the last requested mode.
     */
    m_lastTransponderMode =
        mode;
}


// ============================================================================
// Transponder polling
// ============================================================================

/**
 * @brief Periodically poll and configure the transponder.
 *
 * Called every 200 ms by timerTRANS.
 *
 * The static state variable advances through a sequence of transponder
 * protocol operations.
 *
 * Sequence:
 * - 0 : Send protocol/version setup and request d=g.
 * - 1 : Query z.
 * - 2 : Query altitude.
 * - 3 : Query squawk code.
 * - 4 : Query operating mode.
 * - 5 : Query IDENT state.
 * - 6 : Query protocol value y.
 * - 7 : Query altitude-data mode d.
 * - 8 : If serial altitude mode is active, send current local altitude.
 *
 * After the startup states 0 and 1 have completed, polling loops through
 * states 2 through 8.
 */
void MyTcpSocket::doTransponder()
{
    /*
     * Current polling-state index.
     */
    static int state = 0;


    /*
     * Do not send commands until the transponder interface is connected.
     */
    if (!Transponderstat)
        return;


    switch (state)
    {
    case 0:
        /*
         * Initial protocol/version setup.
         */
        readyWrite(QByteArray("\x02" "v=1" "\x03",5));
        readyWrite(QByteArray("\x02" "d=g" "\x03",5));
        break;

    case 1:
        /*
         * Query identification/version/status text.
         */
        readyWrite(QByteArray("\x02" "z=?" "\x03",5));
        break;

    case 2:
        /*
         * Query altitude.
         */
        readyWrite(QByteArray("\x02" "a=?" "\x03",5));
        break;

    case 3:
        /*
         * Query squawk code.
         */
        readyWrite(QByteArray("\x02" "c=?" "\x03",5));
        break;

    case 4:
        /*
         * Query transponder operating mode.
         */
        readyWrite(QByteArray("\x02" "s=?" "\x03",5));
        break;

    case 5:
        /*
         * Query IDENT state.
         */
        readyWrite(QByteArray("\x02" "i=?" "\x03",5));
        break;

    case 6:
        /*
         * Query protocol-specific y value.
         */
        readyWrite(QByteArray("\x02" "y=?" "\x03",5));
        break;

    case 7:
        /*
         * Query altitude-data mode.
         */
        readyWrite(QByteArray("\x02" "d=?" "\x03",5));
        break;

    case 8:

        /*
         * When the transponder is configured to receive serial altitude and
         * a valid local altitude exists, send the altitude in meters.
         *
         * Example:
         *
         * STX a=123M ETX
         */
        if (transponder_serial_mode && m_altitude > 0.1)
        {
            QByteArray command = QByteArray("\x02" "a=",3) +
                                 QByteArray::number(static_cast<int>(m_altitude)) +
                                 "M" + QByteArray(1,'\x03');

            readyWrite(command);
        }
        break;

    default:
        break;
    }


    /*
     * After startup, continuously repeat states 2 through 8.
     */
    if (++state > 8)
    {
        state = 2;
    }
}


// ============================================================================
// External altimeter parser
// ============================================================================

/**
 * @brief Parse one external-altimeter CSV record.
 *
 * Expected format:
 *
 * @code
 * ALTIMETER,pressure,temperature,relative,altitude,angle
 * @endcode
 *
 * Example:
 *
 * @code
 * ALTIMETER,997.0312,24.2770,0.0898,135.8314,30031033
 * @endcode
 *
 * Processing:
 * - Remove surrounding whitespace.
 * - Split on commas/newlines.
 * - Verify the "ALTIMETER" header.
 * - Parse pressure, temperature, relative value, altitude and angle.
 * - Reject malformed or NaN values.
 * - Store the latest complete measurement.
 * - If external altitude mode is selected, update m_altitude.
 * - Emit externalAltimeterReceived() with a complete copied measurement.
 *
 * @param thiz Target MyTcpSocket object.
 * @param line Received textual altimeter data.
 */
void MyTcpSocket::parseAltimeterLine(
    MyTcpSocket *thiz,
    const QString &line)
{
    if (!thiz)
        return;


    /*
     * Conversion-result flags for each numeric field.
     */
    bool ok1 = false;
    bool ok2 = false;
    bool ok3 = false;
    bool ok4 = false;
    bool ok5 = false;


    /*
     * Remove leading/trailing whitespace and line endings.
     */
    const QString clean =
        line.trimmed();


    /*
     * Accept both comma and newline delimiters because received network or
     * serial buffers may contain line boundaries.
     */
    const QStringList parts =
        clean.split(
            QRegularExpression(
                "[,\n]"),
            Qt::SkipEmptyParts);


    /*
     * A valid record requires at least six fields.
     */
    if (parts.size() < 6)
        return;


    /*
     * Verify the record identifier.
     */
    if (parts[0] !=
        "ALTIMETER")
    {
        return;
    }


    /*
     * Parse pressure.
     */
    const float pressure =
        parts[1].toFloat(
            &ok1);

    /*
     * Parse temperature.
     */
    const float temperature =
        parts[2].toFloat(
            &ok2);

    /*
     * Parse relative value.
     */
    const float relative =
        parts[3].toFloat(
            &ok3);

    /*
     * Parse altitude.
     */
    const float altitude =
        parts[4].toFloat(
            &ok4);

    /*
     * Parse the additional angle/value field.
     *
     * The field is currently validated but otherwise unused.
     */
    const float angle =
        parts[5].toFloat(
            &ok5);

    Q_UNUSED(angle)


    /*
     * Reject records containing fields that cannot be converted to float.
     */
    if (!(ok1 &&
          ok2 &&
          ok3 &&
          ok4 &&
          ok5))
    {
        qWarning()
        << "Invalid altimeter record:"
        << clean;

        return;
    }


    /*
     * Reject invalid IEEE NaN values.
     */
    if (std::isnan(pressure) ||
        std::isnan(temperature) ||
        std::isnan(relative) ||
        std::isnan(altitude))
    {
        qWarning()
        << "NaN in altimeter record:"
        << clean;

        return;
    }


    /*
     * This function runs in the MyTcpSocket thread, therefore these
     * members can be modified without racing MainWindow.
     */
    thiz->Altimeter_data.pressure =
        pressure;

    thiz->Altimeter_data.temperature =
        temperature;

    thiz->Altimeter_data.relative =
        relative;

    thiz->Altimeter_data.altitude =
        altitude;


    /*
     * When EXT altitude mode is selected, the latest external altitude is
     * also the active local altitude used by the transponder subsystem.
     */
    if (thiz->m_transponderAltitudeMode == 1)
    {
        thiz->m_altitude =
            altitude;
    }


    /*
     * MainWindow receives a copied measurement.
     *
     * It does not directly read Altimeter_data while this object updates it.
     */
    emit thiz->externalAltimeterReceived(
        pressure,
        temperature,
        relative,
        altitude);
}


// ============================================================================
// Transponder receive parser
// ============================================================================

/**
 * @brief Parse raw transponder serial bytes.
 *
 * This function is registered as the receive callback for the transponder
 * serial backend.
 *
 * Transponder protocol framing:
 *
 * @code
 * STX payload ETX
 * @endcode
 *
 * where:
 * - STX = 0x02
 * - ETX = 0x03
 *
 * The callback may receive:
 * - A complete frame.
 * - A partial frame.
 * - Several frames in one callback.
 *
 * m_transponderRxBuffer therefore persists between callback invocations.
 *
 * The parser is protected by m_transponderRxMutex because the underlying
 * serial implementation may potentially invoke the callback concurrently.
 *
 * Recognized commands:
 * - s : operating mode.
 * - d : altitude-data mode.
 * - r : annunciator status.
 * - i : IDENT state.
 * - c : squawk code.
 * - a : altitude.
 * - z : text/status/identification.
 * - p : hardware-test state.
 *
 * Parsed values are emitted as signals. MainWindow does not directly access
 * the receive buffer.
 *
 * A '*' character is treated as a transponder ping indication.
 *
 * @param parent Callback context. Expected to point to MyTcpSocket.
 * @param data Raw received byte buffer.
 * @param length Number of valid bytes in @p data.
 */
void MyTcpSocket::ret_transponder(
    void *parent,
    const char *data,
    uint32_t length)
{
    auto *local =
        static_cast<MyTcpSocket *>(parent);


    /*
     * Reject invalid callback input.
     */
    if (!local ||
        !data ||
        length == 0)
    {
        return;
    }


    /*
     * Protect parser state if the serial backend invokes this callback
     * concurrently.
     */
    QMutexLocker<QMutex> locker(
        &local->m_transponderRxMutex);


    /*
     * Process every received byte independently.
     */
    for (uint32_t i = 0;
         i < length;
         ++i)
    {
        const char ch =
            data[i];


        // -----------------------------------------------------------------
        // Ping indication
        // -----------------------------------------------------------------

        /*
         * '*' is treated as a transponder ping/keepalive marker.
         */
        if (ch == '*')
        {
            emit local->transponderPingReceived();

            const QString datalog =
                QDateTime::currentDateTime()
                    .toString(Qt::ISODate) +
                ": Ping received...";

            local->logdata(
                local,
                QString(TRANSPONDERLOG),
                datalog);
        }


        // -----------------------------------------------------------------
        // STX starts a fresh frame
        // -----------------------------------------------------------------

        /*
         * A new STX discards any incomplete previous frame.
         */
        if (ch == STX)
        {
            local->m_transponderRxBuffer.clear();

            continue;
        }


        // -----------------------------------------------------------------
        // ETX completes the frame
        // -----------------------------------------------------------------

        /*
         * ETX marks the current buffer as one complete transponder message.
         */
        if (ch == ETX)
        {
            /*
             * Copy the complete command before clearing the parser buffer.
             */
            const QByteArray command =
                local->m_transponderRxBuffer;

            local->m_transponderRxBuffer.clear();


            /*
             * Responses shorter than three bytes cannot contain the expected
             * "x=value" protocol structure.
             */
            if (command.size() < 3)
            {
                continue;
            }


            /*
             * Create a timestamped diagnostic representation.
             */
            const QString datalog =
                QDateTime::currentDateTime()
                    .toString(Qt::ISODate) +
                ": " +
                QString::fromLocal8Bit(
                    command);

            local->logdata(
                local,
                QString(TRANSPONDERLOG),
                datalog);


            /*
             * All emitted QByteArray arguments are independent copies.
             * MainWindow never reads the parser buffer directly.
             */
            switch (command[0])
            {
            case 's':
            {
                /*
                 * Transponder operating-mode response.
                 */
                emit local->transponderModeReceived(
                    command[2]);

                emit local->transponderActivity();

                break;
            }


            case 'd':
            {
                /*
                 * Altitude-data mode.
                 *
                 * A protocol value of 's' is interpreted as serial altitude
                 * mode.
                 */
                const bool serialMode =
                    command[2] == 's';


                /*
                 * transponder_serial_mode belongs to MyTcpSocket.
                 * Update it in the object's thread rather than directly
                 * in an arbitrary serial callback thread.
                 */
                QMetaObject::invokeMethod(
                    local,
                    [local, serialMode]()
                    {
                        local->transponder_serial_mode =
                            serialMode;

                        emit local->transponderDataModeReceived(
                            serialMode);
                    },
                    Qt::QueuedConnection);

                break;
            }


            case 'r':
            {
                /*
                 * Annunciator/status response.
                 */
                emit local->transponderAnnunciatorReceived(
                    command[2]);

                emit local->transponderActivity();

                break;
            }


            case 'i':
            {
                /*
                 * IDENT state.
                 */
                emit local->transponderIdentReceived(
                    command[2]);

                emit local->transponderActivity();

                break;
            }


            case 'c':
            {
                /*
                 * Squawk-code response.
                 *
                 * The entire command is passed to MainWindow.
                 */
                emit local->transponderCodeReceived(
                    command);

                emit local->transponderActivity();

                break;
            }


            case 'a':
            {
                /*
                 * Altitude response.
                 *
                 * The entire command is passed to MainWindow so units and
                 * value can be processed there.
                 */
                emit local->transponderAltitudeReceived(
                    command);

                emit local->transponderActivity();

                break;
            }


            case 'z':
            {
                /*
                 * Text / status / identification response.
                 */
                emit local->transponderTextReceived(
                    command);

                emit local->transponderActivity();

                break;
            }


            case 'p':
            {
                /*
                 * Hardware-test status.
                 */
                emit local->transponderHardwareStatusReceived(
                    command[2]);

                emit local->transponderActivity();

                break;
            }


            default:
            {
                /*
                 * Preserve unknown commands for diagnostic visibility.
                 */
                qDebug()
                    << "Unhandled transponder command:"
                    << command;

                break;
            }
            }


            /*
             * ETX has been fully handled, so skip payload processing below.
             */
            continue;
        }


        // -----------------------------------------------------------------
        // Ordinary payload character
        // -----------------------------------------------------------------

        /**
         * Maximum amount of payload retained for a single transponder
         * message.
         *
         * This prevents corrupted or unterminated serial data from allowing
         * the receive buffer to grow indefinitely.
         */
        constexpr qsizetype MaxCommandLength =
            128;


        /*
         * Reset the parser if a frame exceeds the expected maximum size.
         */
        if (local->m_transponderRxBuffer.size() >=
            MaxCommandLength)
        {
            qWarning()
            << "Transponder command too long; parser reset";

            local->m_transponderRxBuffer.clear();

            continue;
        }


        /*
         * Append ordinary payload bytes to the current frame.
         */
        local->m_transponderRxBuffer.append(
            ch);
    }
}


// ============================================================================
// Low-level transmit helper
// ============================================================================

/**
 * @brief Send a complete command to the transponder.
 *
 * This function is transport-independent from the caller's point of view.
 *
 * Thread handling:
 * - If called from another thread, the command is copied and queued back to
 *   the thread that owns MyTcpSocket.
 * - QTcpSocket or the serial backend is accessed only after that check.
 *
 * Transport selection:
 * - If m_transponderClient exists, TCP is used.
 * - Otherwise TransponderSerPort is used on non-iOS platforms.
 *
 * If Transponderstat is false, the command is silently ignored.
 *
 * @param data Complete binary command including any required STX/ETX framing.
 */
void MyTcpSocket::readyWrite(
    const QByteArray &data)
{
    /*
     * QTcpSocket and the serial backend must be accessed from this
     * object's owning thread.
     */
    if (QThread::currentThread() !=
        thread())
    {
        /*
         * QByteArray is captured by value so the command remains valid when
         * the queued lambda executes later.
         */
        QMetaObject::invokeMethod(
            this,
            [this, data]()
            {
                readyWrite(data);
            },
            Qt::QueuedConnection);

        return;
    }


    /*
     * Do not transmit until a transponder transport is connected.
     */
    if (!Transponderstat)
        return;


    // ---------------------------------------------------------------------
    // TCP transport
    // ---------------------------------------------------------------------

    /*
     * Prefer the TCP client when one exists.
     */
    if (m_transponderClient)
    {
        if (m_transponderClient->state() ==
            QAbstractSocket::ConnectedState)
        {
            /*
             * Queue the command into QTcpSocket's transmit buffer.
             */
            const qint64 bytes =
                m_transponderClient->write(
                    data);

            /*
             * QIODevice::write() returns -1 on failure.
             */
            if (bytes == -1)
            {
                qWarning()
                << "Transponder TCP write failed:"
                << m_transponderClient->errorString();
            }
        }
        else
        {
            qWarning()
            << "Transponder TCP not connected:"
            << data;
        }

        return;
    }


    // ---------------------------------------------------------------------
    // Serial transport
    // ---------------------------------------------------------------------

#ifndef Q_OS_IOS

    /*
     * Fall back to the serial/USB transponder interface when no TCP client
     * exists.
     */
    if (TransponderSerPort)
    {
        TransponderSerPort->send(
            data);
    }

#endif
}