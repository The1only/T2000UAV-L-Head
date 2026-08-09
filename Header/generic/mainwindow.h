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

// Flight instruments
#include "example/WidgetSix.h"
//  (WidgetSix includes WidgetAI/ALT/ASI/HI/TC/VSI/EADI/EHSI etc.)

#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class SCREEN; }
QT_END_NAMESPACE

// Define the Qiskit interface (used for performance test)
void Qiskit(void);

/**
 * @brief Main UI window for the Glasscockpit 200-UAV.
 *
 * Responsibilities:
 *  - Manage pages (transponder, IMU, primary instruments, glass cockpit, radar,
 *    radio list, autopilot, config, camera).
 *  - Own and configure MyTcpSocket (transponder, IMU/INS, radar, MQTT).
 *  - Interface with onboard sensors (accelerometer, gyro, magnetometer, baro,
 *    orientation, temperature) when external IMU is not present.
 *  - Run EKF-based attitude/heading estimation and propagate to widgets.
 *  - Handle camera capture, GPS updates, and simple flight logging.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // ------------------------------------------------------------------
    // High-level configuration / persistence
    // ------------------------------------------------------------------

    /**
     * @brief Shift next transponder code and append a digit (0..7).
     */
    void addnext(int x);

    /**
     * @brief (Unused helper, kept for compatibility) Add to current code.
     */
    void addcurrent(int x);

    /**
     * @brief Set transponder mode and send to hardware via MyTcpSocket.
     *
     * Mode:
     *  0 = query (s=?)
     *  1 = STBY
     *  2 = ALT ON
     *  3 = ALT/ident
     */
    void setmode(int mode);

    /**
     * @brief Log landing time + duration to log and UI.
     */
    void logLanded();

    /**
     * @brief Log takeoff time to log and UI.
     */
    void logTakeoff();

    /**
     * @brief Initialize camera and permissions (called from ctor).
     */
    void init();

    /**
     * @brief Called when location permission request resolves.
     */
    void permissionUpdated(const QPermission &permission);

    /**
     * @brief Helper to set a pixmap icon on a QPushButton.
     */
    void setButtonIcon(QString iconPath, QPushButton *button);

    /**
     * @brief Compute and apply QNH to match baro altitude with GPS altitude.
     * @return QNH in hPa.
     */
    double setQNH();

    // Set current page...
    void changePage(int direction);

    /// Optional iOS splash screen shown at startup.
    QSplashScreen *splash = nullptr;

    // ------------------------------------------------------------------
    // Attitude / navigation state (public so instruments can be read easily)
    // ------------------------------------------------------------------

    /// Transponder code being set (digits 0..7).
    int next[4]   = {7, 0, 0, 0};

    /// Transponder active code.
    int current[4]= {8, 8, 8, 8};

    /// Current transponder mode (0..3).
    int mode = 0;

    /// Transponder-reported altitude (feet or meters depending on mode).
    double m_tansALT = 0.0;

    /// Ambient temperature [°C].
    qreal  m_temp   = 9999.0;

    /// Misc time variable (seconds).
    double m_ms           = 0.0;

    /// Main hardware IO handler (transponder / radar / INS / MQTT).
    MyTcpSocket *mysocket = nullptr;

    /**
     * @brief (Unused) react to window resize.
     */
    void onResized(int);

    /**
     * @brief Return QScreen on which the widget is currently shown.
     */
    QScreen *getActiveScreen(QWidget *pWidget) const;

    /// Transient "please wait" message during startup.
    NoButtonMessageBox *m_msgBox = nullptr;

    // ------------------------------------------------------------------
    // QML / Charts / Timing
    // ------------------------------------------------------------------
    QQuickView    view;
    QSplineSeries *series = nullptr;
    QElapsedTimer m_timer;


private:
    // ------------------------------------------------------------------
    // Timers
    // ------------------------------------------------------------------
    bool    alt_receiced    = false;
    QTimer *m_Clock         = nullptr;  ///< 1Hz system clock.
    QTimer *timerAlt        = nullptr;  ///< Transponder alt check.
    QTimer *timerPing       = nullptr;  ///< Ping timeout.
    QTimer *timerActive     = nullptr;  ///< Activity timeout.
    QTimer *timerpaint      = nullptr;  ///< (reserved).
    QTimer *m_Display       = nullptr;  ///< Instrument redraw timer.
    QTimer *m_Transponder   = nullptr;  ///< Transponder redraw ...
    // ------------------------------------------------------------------
    // EKF and attitude filter
    // ------------------------------------------------------------------

    int    alt_mode    = 1;       ///< Altitude units mode.
    QSize *m_size      = nullptr; ///< Camera viewfinder size.

    void setalt(int alt_mode);

    int    m_reading   = 0;
    int    m_first     = 0;       ///< Countdown until calibration complete.
    int    m_calibrate = 100;     ///< Remaining calibration cycles.
    bool   m_armed     = false;   ///< Flight timer armed.
    bool   m_takeoff   = false;   ///< True while airborne.
    double m_bearing   = 999.0;   ///< GPS bearing or sentinel.
    double m_heading   = 0.0;     ///< Filtered heading.

    bool   m_use_gps_in_attitude = false;

    // ------------------------------------------------------------------
    // Sensors (QtSensors / QtPositioning)
    // ------------------------------------------------------------------
    // QAltimeterSensor *m_altimeter_sensor; // reserved

    QPressureSensor     *m_pressure_sensor  = nullptr;
    QPressureReading    *m_pressure_reader  = nullptr;

    // ------------------------------------------------------------------
    // Misc state used in EKF / attitude blend
    // ------------------------------------------------------------------
    qreal  m_offset        = 0.0; ///< Pitch offset at calibration.
    bool   m_geopos        = false;
    double m_dt            = 0.0; ///< Time step [s] between EKF updates.

    QTimer *timerTRANS = nullptr; ///< Transponder polling timer.

    /// Calibration message box during IMU calibration.
    NoButtonMessageBox *m_msgBoxCalibrating = nullptr;

    /// Signal emitted on quit/OK (used with dialogs).
    void accepted();

private slots:
    // ------------------------------------------------------------------
    // Simple UI slots / transponder keypad / exit etc.
    // ------------------------------------------------------------------
    /**
     * @brief C-style RX callback from MyTcpSocket for transponder data.
     *
     * @param parent Pointer back to MainWindow instance.
     * @param data   Raw ASCII payload.
     * @param lenght Length of payload in bytes.
     */
    void getTransponderVal();

    // Keypad buttons (set "next" code digit)
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_19_clicked();
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

    // Transponder control
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

    // Misc controls
    void on_reconnect_now_clicked();

    // Status / timers
    void doCheck();
    void reset_ping();
    void active_ping();


public:
    /// Which screen (monitor) index we are using.
    int screen_index = 0;

    /// Qt Designer-generated UI class instance.
    /// NOTE: constructed in a custom way; left unchanged for compatibility.
    Ui::SCREEN *ui = (Ui::SCREEN *) &(*new (Ui::SCREEN));
};

#endif // MAINWINDOW_H
