/**
 * @file mainwindow.cpp
 * @brief Implementation of the main Glasscockpit / transponder UI window.
 *
 * MainWindow is responsible for the graphical presentation and user control
 * of the transponder and altitude subsystem.
 *
 * Communication with MyTcpSocket is signal-driven. Incoming transponder and
 * external-altimeter information is transferred through Qt signals using
 * queued connections rather than by polling shared communication buffers.
 *
 * This provides a clear ownership model:
 *
 * @code
 * Communication thread / callback
 *              |
 *              | emit signal
 *              v
 *       Qt queued connection
 *              |
 *              v
 *        MainWindow thread
 *              |
 *              v
 *          Update GUI
 * @endcode
 *
 * Main responsibilities:
 * - Initialize the user interface.
 * - Initialize the optional platform pressure sensor.
 * - Display transponder operating mode, IDENT state and squawk code.
 * - Display altitude in feet or meters.
 * - Select altitude source:
 *      - TRA  : transponder altitude
 *      - EXT  : external altimeter
 *      - INT  : internal pressure sensor
 *      - AUTO : automatically select a usable source
 * - Maintain communication and altitude watchdog indicators.
 * - Write a startup entry to the flight log.
 * - Recreate the communication subsystem on request.
 *
 * Platform-specific logging:
 * - Android uses SharedStorage / MediaStore.
 * - macOS and iOS use QFile and LOG_DIR.
 */

#include "mainwindow.h"
#include "mytcpsocket.h"

#ifdef Q_OS_ANDROID
#include "sharedstorage.h"
#endif

#include <QApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QIcon>
#include <QLocationPermission>
#include <QPermission>
#include <QPixmap>
#include <QPressureSensor>
#include <QPushButton>
#include <QSensor>
#include <QSplashScreen>
#include <QTimer>

#include <cmath>
#include <cstdio>


/**
 * @brief File-local constants and helper functions.
 *
 * The anonymous namespace prevents these symbols from being exported outside
 * this translation unit.
 */
namespace
{

/**
 * @brief Number of feet in one meter.
 *
 * Used for all altitude conversions between metric and imperial units.
 */
constexpr double FeetPerMeter = 3.2808399;


/**
 * @brief Standard ISA sea-level pressure in hPa.
 *
 * Pressure altitude is referenced to 1013.25 hPa rather than local QNH.
 */
constexpr double StandardPressureHpa = 1013.25;


/**
 * @brief Constant used by the standard pressure-altitude approximation.
 *
 * The resulting altitude is expressed in meters.
 */
constexpr double PressureAltitudeConstantMeters = 44330.0;


/**
 * @brief Exponent used by the pressure-altitude approximation.
 */
constexpr double PressureAltitudeExponent = 0.190284;


/**
 * @brief Interval between altitude-data watchdog checks.
 */
constexpr int AltitudeStatusIntervalMs = 5000;


/**
 * @brief Time communication activity remains indicated after a message.
 */
constexpr int ActivityTimeoutMs = 5000;


/**
 * @brief Time the transponder ping indicator remains active.
 */
constexpr int PingTimeoutMs = 10000;


/*
 * Style-sheet tokens used by the existing Qt Designer button styles.
 *
 * These are substrings within the existing style sheets rather than complete
 * CSS definitions.
 */
constexpr auto ColorInactive  = "1 #888";
constexpr auto ColorGreen     = "1 #2A0";
constexpr auto ColorStatusOk  = "1 #090";
constexpr auto ColorStatusBad = "1 #900";


/**
 * @brief Replace one color token in a push-button style sheet.
 *
 * The style sheet is only changed when @p from is actually present. This
 * avoids unnecessary style-sheet assignments when the button is already in
 * the requested state.
 *
 * @param button Button whose style should be modified.
 * @param from Existing style token.
 * @param to Replacement style token.
 */
void replaceButtonColor(QPushButton *button,
                        const QString &from,
                        const QString &to)
{
    if (!button)
        return;

    QString style = button->styleSheet();

    if (!style.contains(from))
        return;

    style.replace(from, to);

    button->setStyleSheet(style);
}


/**
 * @brief Set a transponder-mode button active or inactive.
 *
 * Active mode buttons use the existing green style token, while inactive
 * buttons use the grey style token.
 *
 * @param button Button to modify.
 * @param active true to mark the button active, false to mark it inactive.
 */
void setModeButtonActive(QPushButton *button,
                         bool active)
{
    if (active)
    {
        replaceButtonColor(
            button,
            ColorInactive,
            ColorGreen);
    }
    else
    {
        replaceButtonColor(
            button,
            ColorGreen,
            ColorInactive);
    }
}


/**
 * @brief Convert altitude from meters to feet.
 *
 * @param meters Altitude in meters.
 * @return Altitude in feet.
 */
double metersToFeet(double meters)
{
    return meters * FeetPerMeter;
}


/**
 * @brief Convert altitude from feet to meters.
 *
 * @param feet Altitude in feet.
 * @return Altitude in meters.
 */
double feetToMeters(double feet)
{
    return feet / FeetPerMeter;
}

} // namespace


/**
 * @brief Construct and initialize the main application window.
 *
 * Startup sequence:
 * - Prepare the platform-specific log directory.
 * - Initialize widgets generated by Qt Designer.
 * - Limit the on-screen diagnostic history.
 * - Optionally show the iOS splash screen.
 * - Initialize current and pending squawk-code displays.
 * - Create MyTcpSocket.
 * - Establish signal/slot communication with MyTcpSocket.
 * - Initialize platform sensors and watchdog timers.
 * - Command the transponder into standby.
 * - Append a startup timestamp to the flight log.
 *
 * @param parent Parent widget, normally nullptr for the main application
 *               window.
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

#if defined(Q_OS_IOS) || defined(Q_OS_MACOS)

    /*
     * macOS and iOS use a conventional filesystem directory.
     */
    const QString documentsPath = LOG_DIR;

    qDebug() << "Log directory:"
             << documentsPath;

    QDir dir(documentsPath);

    /*
     * Create the log directory if it does not already exist.
     */
    if (!dir.exists() &&
        !dir.mkpath("."))
    {
        qWarning()
        << "Could not create log directory:"
        << documentsPath;
    }

#elif defined(Q_OS_ANDROID)

    /*
     * Android public files are written through SharedStorage / MediaStore.
     * No direct QDir creation is necessary here.
     */
    qDebug()
        << "Android logs:"
        << "Download/LowEnergyScanner/";

#endif


    // ---------------------------------------------------------------------
    // User interface
    // ---------------------------------------------------------------------

    /*
     * Instantiate all widgets generated by Qt Designer.
     */
    ui->setupUi(this);

    /*
     * Keep only the newest 50 diagnostic text blocks. QPlainTextEdit removes
     * older blocks automatically as new messages are appended.
     */
    ui->plainTextEdit
        ->document()
        ->setMaximumBlockCount(50);


#if defined(Q_OS_IOS)

    /*
     * Display the startup splash screen on iOS.
     */
    QPixmap splashPixmap(
        ":/images/splash.png");

    splash =
        new QSplashScreen(
            splashPixmap);

    splash->autoFillBackground();
    splash->show();

#endif


    // ---------------------------------------------------------------------
    // Initial transponder display
    // ---------------------------------------------------------------------

    /*
     * Convert the four stored current-code digits into one integer for the
     * LCD display.
     */
    const int currentCode =
        current[0] * 1000 +
        current[1] * 100 +
        current[2] * 10 +
        current[3];

    /*
     * Convert the four pending-code digits into one integer.
     */
    const int nextCode =
        next[0] * 1000 +
        next[1] * 100 +
        next[2] * 10 +
        next[3];

    /*
     * Preserve all four digits, including leading zeroes.
     */
    ui->lcdNumber->display(
        QString::number(currentCode)
            .rightJustified(4, '0'));

    ui->lcdNumber_2->display(
        QString::number(nextCode)
            .rightJustified(4, '0'));

    /*
     * Application identification shown in the diagnostic window.
     */
    ui->plainTextEdit->appendPlainText(
        "Glasscockpit 200-UAV v1.02a");


    // ---------------------------------------------------------------------
    // Communication subsystem
    // ---------------------------------------------------------------------

    /*
     * Create the device communication subsystem.
     *
     * Despite its historical name, MyTcpSocket handles both serial and
     * network-based communication.
     */
    mysocket =
        new MyTcpSocket(
            this,
            ui->plainTextEdit);

    /*
     * Establish all signal/slot connections used for thread-safe data
     * transfer between MyTcpSocket and MainWindow.
     */
    connectTransponderSignals();


    // ---------------------------------------------------------------------
    // Sensors and timers
    // ---------------------------------------------------------------------

    /*
     * Discover the internal pressure sensor and initialize watchdog timers.
     */
    init();


    // ---------------------------------------------------------------------
    // Initial transponder state
    // ---------------------------------------------------------------------

    /*
     * Request standby mode during application startup.
     */
    setmode(1);


    // ---------------------------------------------------------------------
    // Startup logging
    // ---------------------------------------------------------------------

    /*
     * Create the application-start log message.
     */
    const QString data =
        "System booted at: " +
        QDateTime::currentDateTime().toString() +
        '\n';

#ifdef Q_OS_ANDROID

    /*
     * Android logging is performed through SharedStorage, which uses
     * Android MediaStore.
     */
    if (!SharedStorage::appendTextFile(
            "LowEnergyScanner",
            "flightlog.txt",
            data))
    {
        qWarning()
        << "Could not write flight log";
    }

#else

    /*
     * macOS and iOS use a conventional QFile.
     */
    QFile logFile(
        QString(LOG_DIR) +
        QString(FLIGHTLOG));

    if (!logFile.open(
            QIODevice::WriteOnly |
            QIODevice::Append |
            QIODevice::Text))
    {
        qWarning()
        << "Could not open flight log:"
        << logFile.fileName()
        << logFile.errorString();
    }
    else
    {
        logFile.write(
            data.toUtf8());
    }

#endif


    /*
     * Create the startup information box. It is currently constructed but
     * not explicitly shown here.
     */
    m_msgBox =
        new NoButtonMessageBox(
            tr("Please wait for the system to boot!"));
}


/**
 * @brief Destroy the main application window.
 *
 * Qt automatically destroys child QObjects owned by MainWindow.
 */
MainWindow::~MainWindow()
{
    qDebug()
    << "Exiting application";
}


/**
 * @brief Connect the communication subsystem to MainWindow.
 *
 * All receive-side connections use Qt::QueuedConnection. This ensures that
 * slots modifying GUI widgets execute in MainWindow's thread even when the
 * originating signal is emitted from a communication callback or another
 * thread.
 *
 * Two signals are also connected in the opposite direction:
 * - localAltitudeChanged() transfers locally calculated altitude.
 * - transponderAltitudeModeChanged() transfers the selected altitude source.
 *
 * This function must be called again whenever @c mysocket is recreated.
 */
void MainWindow::connectTransponderSignals()
{
    if (!mysocket)
        return;


    /*
     * Generic transponder communication activity.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderActivity,
        this,
        &MainWindow::onTransponderActivity,
        Qt::QueuedConnection);


    /*
     * Transponder ping / keepalive indication.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderPingReceived,
        this,
        &MainWindow::onTransponderPing,
        Qt::QueuedConnection);


    /*
     * Transponder operating-mode response.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderModeReceived,
        this,
        &MainWindow::onTransponderMode,
        Qt::QueuedConnection);


    /*
     * Transponder annunciator response.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderAnnunciatorReceived,
        this,
        &MainWindow::onTransponderAnnunciator,
        Qt::QueuedConnection);


    /*
     * IDENT state.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderIdentReceived,
        this,
        &MainWindow::onTransponderIdent,
        Qt::QueuedConnection);


    /*
     * Squawk-code response.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderCodeReceived,
        this,
        &MainWindow::onTransponderCode,
        Qt::QueuedConnection);


    /*
     * Altitude response.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderAltitudeReceived,
        this,
        &MainWindow::onTransponderAltitude,
        Qt::QueuedConnection);


    /*
     * Text / version / status response.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderTextReceived,
        this,
        &MainWindow::onTransponderText,
        Qt::QueuedConnection);


    /*
     * Hardware self-test result.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderHardwareStatusReceived,
        this,
        &MainWindow::onTransponderHardwareStatus,
        Qt::QueuedConnection);


    /*
     * Transponder altitude-data mode.
     */
    connect(
        mysocket,
        &MyTcpSocket::transponderDataModeReceived,
        this,
        &MainWindow::onTransponderDataMode,
        Qt::QueuedConnection);


    /*
     * External-altimeter measurement.
     */
    connect(
        mysocket,
        &MyTcpSocket::externalAltimeterReceived,
        this,
        &MainWindow::onExternalAltimeter,
        Qt::QueuedConnection);


    /*
     * GUI -> communication subsystem.
     *
     * Transfer a locally selected/calculated altitude without directly
     * writing communication-owned variables.
     */
    connect(
        this,
        &MainWindow::localAltitudeChanged,
        mysocket,
        &MyTcpSocket::setLocalAltitude,
        Qt::QueuedConnection);


    /*
     * Transfer the currently selected altitude source.
     */
    connect(
        this,
        &MainWindow::transponderAltitudeModeChanged,
        mysocket,
        &MyTcpSocket::setTransponderAltitudeMode,
        Qt::QueuedConnection);


    /*
     * Ensure a newly created MyTcpSocket immediately receives the current
     * altitude-source selection.
     */
    emit transponderAltitudeModeChanged(
        m_altitudeSourceMode);
}


/**
 * @brief Assign an icon to a push button.
 *
 * Loads an image from @p iconPath and assigns it to @p button.
 *
 * @param iconPath File or Qt resource path to the image.
 * @param button Destination push button.
 */
void MainWindow::setButtonIcon(
    QString iconPath,
    QPushButton *button)
{
    if (!button)
        return;

    const QPixmap pixmap(iconPath);

    if (pixmap.isNull())
    {
        qWarning()
        << "Could not load icon:"
        << iconPath;

        return;
    }

    button->setIcon(
        QIcon(pixmap));

    button->setIconSize(
        pixmap.size());
}


/**
 * @brief Process completion of a location-permission request.
 *
 * The function requires both general permission approval and precise-location
 * accuracy.
 *
 * @param permission Permission object returned by Qt.
 */
void MainWindow::permissionUpdated(
    const QPermission &permission)
{
    if (permission.status() !=
        Qt::PermissionStatus::Granted)
    {
        qWarning()
        << "Precise location permission denied";

        return;
    }

    const auto locationPermission =
        permission.value<QLocationPermission>();

    if (!locationPermission ||
        locationPermission->accuracy() !=
            QLocationPermission::Precise)
    {
        qWarning()
        << "Precise location unavailable";

        return;
    }

    qDebug()
        << "Precise location granted";
}


/**
 * @brief Initialize available sensors and housekeeping timers.
 *
 * Enumerates Qt sensor types and identifiers for diagnostic purposes.
 *
 * If a QPressureSensor is available:
 * - Create the sensor.
 * - Connect readingChanged() to onPressureReadingChanged().
 * - Request a 4 Hz data rate.
 * - Start acquisition.
 *
 * Timers initialized:
 * - timerPing   : transponder ping watchdog.
 * - timerActive : general communication watchdog.
 * - timerAlt    : altitude-data watchdog.
 */
void MainWindow::init()
{
    /*
     * Enumerate every sensor type exposed by Qt Sensors.
     */
    for (const QByteArray &type :
         QSensor::sensorTypes())
    {
        qDebug()
        << "Sensor type:"
        << type;

        /*
         * Enumerate hardware identifiers associated with this sensor type.
         */
        for (const QByteArray &identifier :
             QSensor::sensorsForType(type))
        {
            qDebug()
            << "    Sensor:"
            << identifier;
        }

        /*
         * Create the internal pressure sensor when available.
         */
        if (type ==
            QByteArrayLiteral(
                "QPressureSensor"))
        {
            m_pressure_sensor =
                new QPressureSensor(this);

            connect(
                m_pressure_sensor,
                &QPressureSensor::readingChanged,
                this,
                &MainWindow::onPressureReadingChanged);

            /*
             * Request approximately four pressure measurements per second.
             */
            m_pressure_sensor->setDataRate(4);

            if (!m_pressure_sensor->start())
            {
                qWarning()
                << "Could not start QPressureSensor";
            }
        }
    }


    /*
     * Ping watchdog.
     *
     * Restarted each time a ping is received.
     */
    timerPing =
        new QTimer(this);

    timerPing->setSingleShot(true);

    connect(
        timerPing,
        &QTimer::timeout,
        this,
        &MainWindow::reset_ping);


    /*
     * General communication watchdog.
     *
     * Restarted whenever a valid transponder response is received.
     */
    timerActive =
        new QTimer(this);

    timerActive->setSingleShot(true);

    connect(
        timerActive,
        &QTimer::timeout,
        this,
        &MainWindow::active_ping);


    /*
     * Altitude watchdog.
     *
     * Periodically verifies that usable altitude data continues to arrive.
     */
    timerAlt =
        new QTimer(this);

    connect(
        timerAlt,
        &QTimer::timeout,
        this,
        &MainWindow::doCheck);

    timerAlt->start(
        AltitudeStatusIntervalMs);
}


/**
 * @brief Process a new internal pressure-sensor reading.
 *
 * Qt reports pressure in Pa. The value is converted to hPa and then converted
 * to ISA pressure altitude in meters using the standard 1013.25 hPa reference.
 *
 * The resulting value is always stored in @c m_internalAltitude. It is sent
 * to MyTcpSocket only when the INT altitude source is currently selected.
 */
void MainWindow::onPressureReadingChanged()
{
    if (!m_pressure_sensor)
        return;

    m_pressure_reader =
        m_pressure_sensor->reading();

    if (!m_pressure_reader)
        return;

    /*
     * Convert Pa -> hPa / mbar.
     */
    const double pressureHpa =
        m_pressure_reader->pressure() /
        100.0;

    /*
     * Calculate ISA pressure altitude in meters.
     */
    m_internalAltitude =
        PressureAltitudeConstantMeters *
        (1.0 -
         std::pow(
             pressureHpa /
                 StandardPressureHpa,
             PressureAltitudeExponent));


    /*
     * Only make the internal pressure altitude the communication
     * subsystem's active local altitude while INT is selected.
     */
    if (m_altitudeSourceMode == 3)
    {
        emit localAltitudeChanged(
            m_internalAltitude);
    }
}


/**
 * @brief Process a complete external-altimeter measurement.
 *
 * The current MainWindow logic requires only altitude, but the complete
 * measurement is delivered by the signal for future use.
 *
 * When EXT is selected, the external altitude is forwarded to MyTcpSocket as
 * the active locally supplied altitude.
 *
 * @param pressure External altimeter pressure value.
 * @param temperature External altimeter temperature value.
 * @param relative External altimeter relative value.
 * @param altitude External altitude, expected in meters.
 */
void MainWindow::onExternalAltimeter(
    float pressure,
    float temperature,
    float relative,
    float altitude)
{
    Q_UNUSED(pressure)
    Q_UNUSED(temperature)
    Q_UNUSED(relative)

    /*
     * Keep a GUI-thread-owned copy of the latest external altitude.
     */
    m_externalAltitude =
        altitude;

    /*
     * If EXT is selected, this becomes the active altitude supplied
     * back to the transponder subsystem.
     */
    if (m_altitudeSourceMode == 1)
    {
        emit localAltitudeChanged(
            m_externalAltitude);
    }
}


/**
 * @brief Handle valid transponder communication activity.
 *
 * Changes the communication-status indicator to its healthy state and
 * restarts the activity watchdog.
 */
void MainWindow::onTransponderActivity()
{
    replaceButtonColor(
        ui->pushButton_14,
        ColorStatusBad,
        ColorStatusOk);

    timerActive->start(
        ActivityTimeoutMs);
}


/**
 * @brief Handle reception of a transponder ping.
 *
 * Marks the ping indicator healthy and restarts its watchdog timer.
 */
void MainWindow::onTransponderPing()
{
    replaceButtonColor(
        ui->pushButton_10,
        ColorStatusBad,
        ColorStatusOk);

    timerPing->start(
        PingTimeoutMs);
}


/**
 * @brief Process a transponder operating-mode response.
 *
 * Protocol values currently interpreted:
 * - 't' : standby.
 * - 'a' : Mode A.
 * - 'c' : Mode C / altitude.
 *
 * Only updates the button styles when the reported mode differs from the
 * currently displayed mode.
 *
 * @param receivedMode Protocol mode character.
 */
void MainWindow::onTransponderMode(
    char receivedMode)
{
    int newMode = mode;

    switch (receivedMode)
    {
    case 't':
        newMode = 1;
        break;

    case 'a':
        newMode = 2;
        break;

    case 'c':
        newMode = 3;
        break;

    default:
        qWarning()
            << "Unknown transponder mode:"
            << receivedMode;

        return;
    }

    /*
     * Avoid unnecessary GUI style updates.
     */
    if (newMode == mode)
        return;

    setModeButtonActive(
        ui->pushButton_stby,
        newMode == 1);

    setModeButtonActive(
        ui->pushButton_norm,
        newMode == 2);

    setModeButtonActive(
        ui->pushButton_alt,
        newMode == 3);

    mode = newMode;
}


/**
 * @brief Process transponder annunciator state.
 *
 * A protocol value of 'N' is interpreted as false; all other values are
 * currently interpreted as true.
 *
 * @param value Annunciator protocol character.
 */
void MainWindow::onTransponderAnnunciator(
    char value)
{
    const bool state =
        value != 'N';

    qDebug()
        << "Annunciator:"
        << state;
}


/**
 * @brief Process transponder IDENT state.
 *
 * A value of '0' means IDENT is inactive. Other values are treated as active.
 * The IDENT button style is changed accordingly.
 *
 * @param value IDENT protocol character.
 */
void MainWindow::onTransponderIdent(
    char value)
{
    const bool identActive =
        value != '0';

    if (identActive)
    {
        replaceButtonColor(
            ui->pushButton_Ident,
            ColorInactive,
            ColorStatusBad);
    }
    else
    {
        replaceButtonColor(
            ui->pushButton_Ident,
            ColorStatusBad,
            ColorInactive);
    }
}


/**
 * @brief Process a squawk-code response from the transponder.
 *
 * Expected response format:
 *
 * @code
 * c=7000
 * @endcode
 *
 * The parsed value is formatted as four digits and copied into the current[]
 * digit array before updating the LCD.
 *
 * @param command Complete transponder response.
 */
void MainWindow::onTransponderCode(
    const QByteArray &command)
{
    int number = 0;

    /*
     * Parse the numeric part of "c=XXXX".
     */
    if (std::sscanf(
            command.constData(),
            "c=%d",
            &number) != 1)
    {
        qWarning()
        << "Invalid squawk response:"
        << command;

        return;
    }

    /*
     * Preserve leading zeroes in the displayed code.
     */
    const QString code =
        QString::number(number)
            .rightJustified(4, '0');

    if (code.length() != 4)
        return;

    /*
     * Store the individual code digits.
     */
    current[0] =
        code[0].digitValue();

    current[1] =
        code[1].digitValue();

    current[2] =
        code[2].digitValue();

    current[3] =
        code[3].digitValue();

    /*
     * Update the currently active squawk display.
     */
    ui->lcdNumber->display(code);
}


/**
 * @brief Process altitude reported by the transponder.
 *
 * Expected protocol examples:
 *
 * @code
 * a=1234F
 * a=375M
 * @endcode
 *
 * Processing sequence:
 * 1. Parse the received numeric altitude.
 * 2. Normalize the transponder value to meters.
 * 3. In AUTO mode, detect an invalid/implausible transponder altitude.
 * 4. If necessary, switch to EXT or INT.
 * 5. Convert the selected altitude into the user-selected display unit.
 * 6. Round feet to 100-foot increments.
 * 7. Update the LCD, unit label and altitude watchdog.
 *
 * @param command Complete altitude response.
 */
void MainWindow::onTransponderAltitude(
    const QByteArray &command)
{
    /*
     * Work on a local copy so the original signal argument remains unchanged.
     */
    QByteArray data = command;

    float receivedAltitude = 0.0F;

    /*
     * Preserve the previous protocol fallback.
     *
     * A query-style response is converted into a one-meter value so the
     * existing processing path can continue.
     */
    if (data.size() > 2 &&
        data[2] == '?')
    {
        data = "a=1M";
    }


    /*
     * Extract numeric altitude.
     */
    if (std::sscanf(
            data.constData(),
            "a=%f",
            &receivedAltitude) != 1)
    {
        qWarning()
        << "Invalid altitude response:"
        << data;

        return;
    }


    /*
     * Determine whether the transponder value is in feet.
     *
     * Values without 'F' are currently treated as meters.
     */
    const bool reportedFeet =
        data.contains('F');

    /*
     * Normalize all transponder altitude values to meters.
     */
    double altitudeMeters =
        receivedAltitude;

    if (reportedFeet)
    {
        altitudeMeters =
            feetToMeters(
                altitudeMeters);
    }


    /*
     * AUTO mode:
     *
     * A transponder altitude below 0.1 m or above 5000 m is currently treated
     * as unusable. When that occurs, the system first tries the external
     * altimeter and then the internal pressure sensor.
     */
    if (m_altitudeSourceMode == 2 &&
        (altitudeMeters < 0.1 ||
         altitudeMeters > 5000.0))
    {
        /*
         * First choice: external altitude sensor.
         */
        if (m_externalAltitude > 0.1)
        {
            m_altitudeSourceMode = 1;

            /*
             * Inform the communication subsystem that EXT is now active.
             */
            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            /*
             * Configure transponder altitude-data handling.
             */
            mysocket->TransponderMode(false);

            ui->pushButton_27->setText(
                "EXT");

            altitudeMeters =
                m_externalAltitude;

            /*
             * Supply the external altitude to MyTcpSocket.
             */
            emit localAltitudeChanged(
                altitudeMeters);

            /*
             * Orange identifies the external altitude source.
             */
            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: orange; }");
        }

        /*
         * Second choice: internal pressure sensor.
         */
        else if (m_pressure_sensor &&
                 m_internalAltitude > 0.1)
        {
            m_altitudeSourceMode = 3;

            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            mysocket->TransponderMode(false);

            ui->pushButton_27->setText(
                "INT");

            altitudeMeters =
                m_internalAltitude;

            emit localAltitudeChanged(
                altitudeMeters);

            /*
             * Magenta identifies the internal pressure-sensor source.
             */
            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: #FF00FF; }");
        }
    }


    QString altitudeType;
    double displayAltitude = 0.0;


    /*
     * Convert normalized altitude into the selected display units.
     */
    if (alt_mode == 1)
    {
        /*
         * Feet display.
         */
        displayAltitude =
            metersToFeet(
                altitudeMeters);

        /*
         * Display transponder altitude in 100-foot increments.
         */
        displayAltitude =
            std::round(
                displayAltitude /
                100.0) *
            100.0;

        altitudeType =
            "Alt.Ft.";
    }
    else
    {
        /*
         * Meter display.
         */
        displayAltitude =
            altitudeMeters;

        altitudeType =
            "Alt.M.";
    }


    /*
     * Store the rounded value currently shown by the UI.
     */
    m_tansALT =
        std::round(
            displayAltitude);


    /*
     * Display a minimum of four digits, padding with leading zeroes.
     */
    const QString altitudeText =
        QString::number(
            static_cast<int>(
                m_tansALT))
            .rightJustified(4, '0');


    /*
     * Update altitude LCD and unit label.
     */
    ui->lcdNumber_3->display(
        altitudeText);

    ui->label_2->setText(
        altitudeType);

    /*
     * Add altitude information to the short diagnostic history.
     */
    ui->plainTextEdit->appendPlainText(
        "Altitude to display: " +
        altitudeText);


    /*
     * Mark altitude data as alive for the watchdog.
     *
     * The current design considers only positive displayed altitude valid.
     */
    if (m_tansALT > 0)
    {
        alt_received = true;
    }
}


/**
 * @brief Process a text/status response from the transponder.
 *
 * The protocol prefix occupies the first two bytes. The remaining payload is
 * displayed in the diagnostic text window.
 *
 * @param command Complete response.
 */
void MainWindow::onTransponderText(
    const QByteArray &command)
{
    if (command.size() <= 2)
        return;

    ui->plainTextEdit->appendPlainText(
        QString::fromLocal8Bit(
            command.constData() + 2,
            command.size() - 2));
}


/**
 * @brief Process transponder hardware-test status.
 *
 * A value other than '1' is currently interpreted as a successful hardware
 * state.
 *
 * The diagnostic text window is updated and the status/ping indicator is
 * changed accordingly.
 *
 * @param value Hardware-test protocol value.
 */
void MainWindow::onTransponderHardwareStatus(
    char value)
{
    const bool hardwareOk =
        value != '1';

    ui->plainTextEdit->appendPlainText(
        tr("Hardware test status: %1")
            .arg(hardwareOk));

    if (hardwareOk)
    {
        /*
         * Show healthy state and retain it for the watchdog interval.
         */
        replaceButtonColor(
            ui->pushButton_10,
            ColorStatusBad,
            ColorStatusOk);

        timerPing->start(
            ActivityTimeoutMs);
    }
    else
    {
        /*
         * Show failed/unhealthy state.
         */
        replaceButtonColor(
            ui->pushButton_10,
            ColorStatusOk,
            ColorStatusBad);
    }
}


/**
 * @brief Process the transponder altitude-data mode.
 *
 * Currently used for diagnostic output only.
 *
 * @param serialMode true when the transponder reports serial altitude mode.
 */
void MainWindow::onTransponderDataMode(
    bool serialMode)
{
    qDebug()
    << "Transponder serial altitude mode:"
    << serialMode;
}


/**
 * @brief Handle expiration of the transponder ping watchdog.
 *
 * Marks the ping/status indicator as unhealthy.
 */
void MainWindow::reset_ping()
{
    replaceButtonColor(
        ui->pushButton_10,
        ColorStatusOk,
        ColorStatusBad);
}


/**
 * @brief Handle expiration of the general communication watchdog.
 *
 * Marks the communication activity indicator as inactive/unhealthy.
 */
void MainWindow::active_ping()
{
    replaceButtonColor(
        ui->pushButton_14,
        ColorStatusOk,
        ColorStatusBad);
}


/**
 * @brief Periodically verify that altitude data is still being received.
 *
 * If @c alt_received was set during the previous watchdog interval, the
 * altitude status indicator is marked healthy. Otherwise it is marked bad.
 *
 * The flag is reset after each check and must be set again by a later
 * altitude update.
 */
void MainWindow::doCheck()
{
    if (alt_received)
    {
        replaceButtonColor(
            ui->pushButton_11,
            ColorStatusBad,
            ColorStatusOk);
    }
    else
    {
        replaceButtonColor(
            ui->pushButton_11,
            ColorStatusOk,
            ColorStatusBad);
    }

    alt_received = false;
}


/**
 * @brief Select altitude display units.
 *
 * @param mode
 *        0 = meters.
 *        1 = feet.
 */
void MainWindow::setalt(
    int mode)
{
    alt_mode = mode;

    qDebug()
        << "Altitude display:"
        << (alt_mode == 1
                ? "feet"
                : "meters");
}


/**
 * @brief Close the main application window.
 */
void MainWindow::accepted()
{
    close();
}


/**
 * @brief Append one digit to the pending squawk code.
 *
 * Transponder codes use octal digits, so only values from 0 through 7 are
 * accepted.
 *
 * Existing digits are shifted one place to the left and the supplied digit
 * becomes the least-significant digit.
 *
 * @param digit Digit in the range 0...7.
 */
void MainWindow::addnext(
    int digit)
{
    if (digit < 0 ||
        digit > 7)
    {
        return;
    }

    /*
     * Shift existing code left by one digit.
     */
    next[0] = next[1];
    next[1] = next[2];
    next[2] = next[3];
    next[3] = digit;

    /*
     * Reconstruct the four-digit integer representation.
     */
    const int code =
        next[0] * 1000 +
        next[1] * 100 +
        next[2] * 10 +
        next[3];

    /*
     * Display with leading zeroes.
     */
    ui->lcdNumber_2->display(
        QString::number(code)
            .rightJustified(4, '0'));
}


/**
 * @brief Send or query a transponder operating mode.
 *
 * Protocol commands:
 * - 0 -> STX s=? ETX
 * - 1 -> STX s=t ETX
 * - 2 -> STX s=a ETX
 * - 3 -> STX s=c ETX
 *
 * @param requestedMode Requested operating mode.
 */
void MainWindow::setmode(
    int requestedMode)
{
    if (!mysocket)
        return;

    switch (requestedMode)
    {
    case 0:
        mysocket->readyWrite(
            QByteArray(
                "\x02s=?\x03",
                5));
        break;

    case 1:
        mysocket->readyWrite(
            QByteArray(
                "\x02s=t\x03",
                5));
        break;

    case 2:
        mysocket->readyWrite(
            QByteArray(
                "\x02s=a\x03",
                5));
        break;

    case 3:
        mysocket->readyWrite(
            QByteArray(
                "\x02s=c\x03",
                5));
        break;

    default:
        qWarning()
            << "Invalid transponder mode:"
            << requestedMode;
        break;
    }
}


// ============================================================================
// Squawk keypad
// ============================================================================

/**
 * @brief Append digit 1 to the pending squawk code.
 */
void MainWindow::on_pushButton_clicked()
{
    addnext(1);
}

/**
 * @brief Append digit 2 to the pending squawk code.
 */
void MainWindow::on_pushButton_2_clicked()
{
    addnext(2);
}

/**
 * @brief Append digit 3 to the pending squawk code.
 */
void MainWindow::on_pushButton_3_clicked()
{
    addnext(3);
}

/**
 * @brief Append digit 4 to the pending squawk code.
 */
void MainWindow::on_pushButton_4_clicked()
{
    addnext(4);
}

/**
 * @brief Append digit 5 to the pending squawk code.
 */
void MainWindow::on_pushButton_5_clicked()
{
    addnext(5);
}

/**
 * @brief Append digit 6 to the pending squawk code.
 */
void MainWindow::on_pushButton_6_clicked()
{
    addnext(6);
}

/**
 * @brief Append digit 7 to the pending squawk code.
 */
void MainWindow::on_pushButton_7_clicked()
{
    addnext(7);
}

/**
 * @brief Digit 8 handler.
 *
 * Intentionally performs no action because transponder squawk codes are
 * octal and cannot contain 8.
 */
void MainWindow::on_pushButton_8_clicked()
{
}

/**
 * @brief Digit 9 handler.
 *
 * Intentionally performs no action because transponder squawk codes are
 * octal and cannot contain 9.
 */
void MainWindow::on_pushButton_9_clicked()
{
}

/**
 * @brief Append digit 0 to the pending squawk code.
 */
void MainWindow::on_pushButton_17_clicked()
{
    addnext(0);
}


// ============================================================================
// Window controls
// ============================================================================

/**
 * @brief Close the main window.
 */
void MainWindow::on_exit_2_clicked()
{
    close();
}

/**
 * @brief Close the main window using the alternate exit control.
 */
void MainWindow::on_pushButton_19_clicked()
{
    close();
}


/**
 * @brief Program the pending squawk code into the transponder.
 *
 * Copies next[] into current[], constructs a framed c=XXXX command and sends
 * it through MyTcpSocket.
 */
void MainWindow::on_pushButton_16_clicked()
{
    /*
     * Copy the selected code into the current-code array.
     */
    current[0] = next[0];
    current[1] = next[1];
    current[2] = next[2];
    current[3] = next[3];

    /*
     * Reconstruct the numeric squawk code.
     */
    const int code =
        next[0] * 1000 +
        next[1] * 100 +
        next[2] * 10 +
        next[3];

    /*
     * Build:
     *
     * STX c=XXXX ETX
     */
    QByteArray command =
        QByteArray("\x02" "c=", 3) +
        QByteArray::number(code) +
        QByteArray(1, '\x03');

    qDebug()
        << "Set squawk:"
        << code;

    mysocket->readyWrite(
        command);
}


/**
 * @brief Set and transmit squawk code 7000.
 *
 * Sets the pending code to 7000, updates the LCD and immediately sends the
 * corresponding framed transponder command.
 */
void MainWindow::on_pushButton_18_clicked()
{
    next[0] = 7;
    next[1] = 0;
    next[2] = 0;
    next[3] = 0;

    ui->lcdNumber_2->display(
        "7000");

    mysocket->readyWrite(
        QByteArray(
            "\x02c=7000\x03",
            8));
}


/**
 * @brief Activate transponder IDENT.
 *
 * Sends:
 *
 * @code
 * STX i=s ETX
 * @endcode
 */
void MainWindow::on_pushButton_Ident_clicked()
{
    mysocket->readyWrite(
        QByteArray(
            "\x02i=s\x03",
            5));
}


/**
 * @brief Cycle between available altitude sources.
 *
 * Source sequence:
 *
 * @code
 * TRA -> EXT -> INT -> AUTO -> TRA
 * @endcode
 *
 * Source meaning:
 * - TRA  : altitude supplied by the transponder.
 * - EXT  : external serial/network altimeter.
 * - INT  : platform internal pressure sensor.
 * - AUTO : use transponder altitude unless it becomes invalid, then fall
 *          back to EXT and finally INT.
 *
 * Display colors currently used:
 * - EXT : orange.
 * - INT : magenta.
 * - unavailable requested source : red.
 * - TRA/AUTO : cyan.
 */
void MainWindow::on_pushButton_27_clicked()
{
    const QString currentSource =
        ui->pushButton_27->text();


    // ---------------------------------------------------------------------
    // TRA -> EXT
    // ---------------------------------------------------------------------

    if (currentSource == "TRA")
    {
        /*
         * Advance button state to EXT.
         */
        ui->pushButton_27->setText(
            "EXT");

        /*
         * External altitude is available.
         */
        if (m_externalAltitude > 0.1)
        {
            m_altitudeSourceMode = 1;

            /*
             * Inform MyTcpSocket of the selected source.
             */
            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            /*
             * Supply the latest external altitude.
             */
            emit localAltitudeChanged(
                m_externalAltitude);

            /*
             * Configure the transponder for externally supplied altitude.
             */
            mysocket->TransponderMode(
                false);

            /*
             * Orange identifies the external sensor.
             */
            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: orange; }");
        }
        else
        {
            /*
             * EXT requested but no usable external altitude currently exists.
             * Retain transponder altitude internally and indicate the
             * unavailable selection in red.
             */
            m_altitudeSourceMode = 0;

            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            mysocket->TransponderMode(
                true);

            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: red; }");
        }

        return;
    }


    // ---------------------------------------------------------------------
    // EXT -> INT
    // ---------------------------------------------------------------------

    if (currentSource == "EXT")
    {
        /*
         * Advance button state to INT.
         */
        ui->pushButton_27->setText(
            "INT");

        /*
         * Use internal pressure sensor when available.
         */
        if (m_pressure_sensor)
        {
            m_altitudeSourceMode = 3;

            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            /*
             * Send the latest pressure-derived altitude.
             */
            emit localAltitudeChanged(
                m_internalAltitude);

            mysocket->TransponderMode(
                false);

            /*
             * Magenta identifies the internal sensor.
             */
            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: #FF00FF; }");
        }
        else
        {
            /*
             * No internal pressure sensor is available.
             */
            m_altitudeSourceMode = 0;

            emit transponderAltitudeModeChanged(
                m_altitudeSourceMode);

            mysocket->TransponderMode(
                true);

            ui->lcdNumber_3->setStyleSheet(
                "QLCDNumber { color: red; }");
        }

        return;
    }


    // ---------------------------------------------------------------------
    // INT -> AUTO
    // ---------------------------------------------------------------------

    if (currentSource == "INT")
    {
        /*
         * Enable automatic source handling.
         */
        m_altitudeSourceMode = 2;

        emit transponderAltitudeModeChanged(
            m_altitudeSourceMode);

        mysocket->TransponderMode(
            true);

        ui->pushButton_27->setText(
            "AUTO");

        /*
         * Cyan identifies normal transponder/AUTO operation.
         */
        ui->lcdNumber_3->setStyleSheet(
            "QLCDNumber { color: rgb(13, 255, 252); }");

        return;
    }


    // ---------------------------------------------------------------------
    // AUTO -> TRA
    // ---------------------------------------------------------------------

    /*
     * Any remaining state returns to normal transponder altitude.
     */
    m_altitudeSourceMode = 0;

    emit transponderAltitudeModeChanged(
        m_altitudeSourceMode);

    mysocket->TransponderMode(
        true);

    ui->pushButton_27->setText(
        "TRA");

    ui->lcdNumber_3->setStyleSheet(
        "QLCDNumber { color: rgb(13, 255, 252); }");
}


// ============================================================================
// Transponder mode buttons
// ============================================================================

/**
 * @brief Request transponder standby mode.
 */
void MainWindow::on_pushButton_stby_clicked()
{
    setmode(1);
}

/**
 * @brief Request Mode A operation.
 */
void MainWindow::on_pushButton_norm_clicked()
{
    setmode(2);
}

/**
 * @brief Request Mode C / altitude operation.
 */
void MainWindow::on_pushButton_alt_clicked()
{
    setmode(3);
}


// ============================================================================
// Altitude display units
// ============================================================================

/**
 * @brief Select altitude display in meters.
 */
void MainWindow::on_pushButton_12_clicked()
{
    setalt(0);
}

/**
 * @brief Select altitude display in feet.
 */
void MainWindow::on_pushButton_13_clicked()
{
    setalt(1);
}


/**
 * @brief Request transponder hardware-test status.
 *
 * Sends:
 *
 * @code
 * STX p=? ETX
 * @endcode
 */
void MainWindow::on_pushButton_off_clicked()
{
    mysocket->readyWrite(
        QByteArray(
            "\x02p=?\x03",
            5));
}


/**
 * @brief Recreate the communication subsystem.
 *
 * Deletes the existing MyTcpSocket object, creates a replacement and then
 * reconnects all signal/slot paths used by MainWindow.
 *
 * This provides a manual recovery path when the communication backend needs
 * to be reinitialized.
 */
void MainWindow::on_reconnect_now_clicked()
{
    delete mysocket;

    mysocket =
        new MyTcpSocket(
            this,
            ui->plainTextEdit);

    /*
     * The new object has no connections inherited from the deleted instance,
     * so reconnect all communication signals.
     */
    connectTransponderSignals();
}