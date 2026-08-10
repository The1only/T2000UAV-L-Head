#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QQuickView>

#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>

#include <QElapsedTimer>
#include <QCameraDevice>
#include <QMediaDevices>
#include <QCamera>
#include <QMediaCaptureSession>
#include <QMediaRecorder>
#include <QImageCapture>

#include <QMessageBox>
#include <QSplashScreen>
#include <QTimer>
#include <QScreen>
#include <QSize>
#include <QDateTime>
#include <QPushButton>
#include <QPermission>
#include <QByteArray>

#include <QGyroscope>
#include <QGyroscopeReading>

#include <QAccelerometer>
#include <QAccelerometerReading>

#include <QCompass>
#include <QCompassReading>

#include <QMagnetometer>
#include <QMagnetometerReading>

#include <QOrientationSensor>
#include <QOrientationReading>

#include <QAmbientTemperatureSensor>
#include <QAmbientTemperatureReading>

#include <QRotationSensor>
#include <QPressureSensor>
#include <QPressureReading>

#include <QGeoPositionInfo>
#include <QGeoCoordinate>
#include <QGeoPositionInfoSource>

#include "mytcpsocket.h"

#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif


QT_BEGIN_NAMESPACE
namespace Ui
{
class SCREEN;
}
QT_END_NAMESPACE


// Qiskit interface used for performance testing.
void Qiskit(void);


/**
 * @brief Main UI window for the Glasscockpit 200-UAV application.
 *
 * MainWindow owns and updates the graphical user interface.
 *
 * Communication data from MyTcpSocket is transferred using Qt signals
 * and slots rather than by polling shared receive buffers. Queued
 * connections ensure that GUI updates execute in the MainWindow thread.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    // =====================================================================
    // Transponder controls
    // =====================================================================

    /**
     * @brief Set an icon on a push button.
     *
     * @param iconPath Path or Qt resource path to the icon.
     * @param button Button receiving the icon.
     */
    void setButtonIcon(QString iconPath, QPushButton *button);

    /**
     * @brief Shift the pending squawk code and append one digit.
     *
     * @param x Digit in the range 0...7.
     */
    void addnext(int x);

    /**
     * @brief Historical helper retained for compatibility.
     */
    void addcurrent(int x);

    /**
     * @brief Set or query the transponder operating mode.
     *
     * @param mode
     *        0 = Query.
     *        1 = Standby.
     *        2 = Mode A.
     *        3 = Mode C.
     */
    void setmode(int mode);


    // =====================================================================
    // Flight logging
    // =====================================================================

    void logLanded();
    void logTakeoff();


    // =====================================================================
    // Initialization
    // =====================================================================

    /**
     * @brief Initialize sensors and housekeeping timers.
     */
    void init();

    /**
     * @brief Process completion of a Qt permission request.
     */
    void permissionUpdated(const QPermission &permission);

    /**
     * @brief Calculate QNH from available pressure/GPS information.
     *
     * @return QNH in hPa.
     */
    double setQNH();

    /**
     * @brief Change the displayed application page.
     */
    void changePage(int direction);


    /// Optional startup splash screen.
    QSplashScreen *splash = nullptr;


    // =====================================================================
    // Public application state
    // =====================================================================

    /// Pending four-digit transponder squawk code.
    int next[4] = {7, 0, 0, 0};

    /// Current transponder squawk code.
    int current[4] = {8, 8, 8, 8};

    /// Current transponder operating mode.
    int mode = 0;

    /// Altitude currently shown to the user.
    double m_tansALT = 0.0;

    /// Ambient temperature [°C].
    qreal m_temp = 9999.0;

    /// Miscellaneous time variable.
    double m_ms = 0.0;

    /// Main communication subsystem.
    MyTcpSocket *mysocket = nullptr;

    void onResized(int);

    QScreen *getActiveScreen(QWidget *pWidget) const;

    /// Startup information dialog.
    NoButtonMessageBox *m_msgBox = nullptr;


    // =====================================================================
    // QML / charts / timing
    // =====================================================================

    QQuickView view;

    QSplineSeries *series = nullptr;

    QElapsedTimer m_timer;


signals:

    /**
     * @brief Transfer locally calculated altitude to MyTcpSocket.
     *
     * Connected using Qt::QueuedConnection so that m_altitude inside
     * MyTcpSocket is modified only in the MyTcpSocket thread.
     *
     * @param altitudeMeters Altitude in meters.
     */
    void localAltitudeChanged(double altitudeMeters);


    /**
     * @brief Transfer selected altitude-source mode to MyTcpSocket.
     *
     * @param mode
     *        0 = Transponder.
     *        1 = External altimeter.
     *        2 = Automatic.
     *        3 = Internal pressure sensor.
     */
    void transponderAltitudeModeChanged(int mode);


private:

    // =====================================================================
    // Internal helpers
    // =====================================================================

    /**
     * @brief Connect all MyTcpSocket signals to MainWindow.
     *
     * Must be called whenever mysocket is recreated.
     */
    void connectTransponderSignals();


    /**
     * @brief Set the altitude display unit.
     *
     * @param alt_mode
     *        0 = meters.
     *        1 = feet.
     */
    void setalt(int alt_mode);


    // =====================================================================
    // Altitude state
    // =====================================================================

    /**
     * @brief True when valid altitude was received during the current
     * watchdog interval.
     */
    bool alt_received = false;

    /**
     * @brief Selected altitude source.
     *
     * 0 = TRA
     * 1 = EXT
     * 2 = AUTO
     * 3 = INT
     */
    int m_altitudeSourceMode = 0;

    /**
     * @brief Altitude display unit.
     *
     * 0 = meters
     * 1 = feet
     */
    int alt_mode = 1;

    /// Latest external-altimeter altitude [m].
    double m_externalAltitude = 0.0;

    /// Latest internal pressure-sensor altitude [m].
    double m_internalAltitude = 0.0;


    // =====================================================================
    // Timers
    // =====================================================================

    QTimer *m_Clock     = nullptr; ///< 1 Hz system clock.
    QTimer *timerAlt    = nullptr; ///< Altitude watchdog.
    QTimer *timerPing   = nullptr; ///< Ping timeout.
    QTimer *timerActive = nullptr; ///< Communication timeout.
    QTimer *timerpaint  = nullptr; ///< Reserved.
    QTimer *m_Display   = nullptr; ///< Instrument redraw.


    // =====================================================================
    // Miscellaneous application state
    // =====================================================================

    QSize *m_size = nullptr;

    int m_reading   = 0;
    int m_first     = 0;
    int m_calibrate = 100;

    bool m_armed   = false;
    bool m_takeoff = false;

    double m_bearing = 999.0;
    double m_heading = 0.0;

    bool m_use_gps_in_attitude = false;


    // =====================================================================
    // Sensors
    // =====================================================================

    QPressureSensor  *m_pressure_sensor = nullptr;
    QPressureReading *m_pressure_reader = nullptr;


    // =====================================================================
    // Navigation / filter state
    // =====================================================================

    qreal m_offset = 0.0;

    bool m_geopos = false;

    double m_dt = 0.0;

    QTimer *timerTRANS = nullptr;

    NoButtonMessageBox *m_msgBoxCalibrating = nullptr;

    void accepted();


private slots:

    // =====================================================================
    // Thread-safe communication receive slots
    // =====================================================================

    void onTransponderActivity();

    void onTransponderPing();

    void onTransponderMode(char mode);

    void onTransponderAnnunciator(char value);

    void onTransponderIdent(char value);

    void onTransponderCode(const QByteArray &command);

    void onTransponderAltitude(const QByteArray &command);

    void onTransponderText(const QByteArray &command);

    void onTransponderHardwareStatus(char value);

    void onTransponderDataMode(bool serialMode);


    /**
     * @brief Receive an external-altimeter measurement.
     */
    void onExternalAltimeter(float pressure,
                             float temperature,
                             float relative,
                             float altitude);


    // =====================================================================
    // UI controls
    // =====================================================================

    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_17_clicked();
    void on_pushButton_16_clicked();
    void on_pushButton_18_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_Ident_clicked();

    void on_pushButton_stby_clicked();
    void on_pushButton_off_clicked();
    void on_pushButton_norm_clicked();
    void on_pushButton_alt_clicked();

    void on_pushButton_12_clicked();
    void on_pushButton_13_clicked();

    void on_pushButton_27_clicked();

    void on_exit_2_clicked();

    void onPressureReadingChanged();

    void on_reconnect_now_clicked();

    void doCheck();
    void reset_ping();
    void active_ping();


public:

    /// Screen/monitor index currently in use.
    int screen_index = 0;

    /**
     * @brief Qt Designer-generated UI instance.
     *
     * Existing allocation retained for compatibility.
     */
    Ui::SCREEN *ui =
        (Ui::SCREEN *) &(*new (Ui::SCREEN));
};

#endif // MAINWINDOW_H