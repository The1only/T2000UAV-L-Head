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
#include "ekfNavINS.h"
#include "mqttclient.h"

// Flight instruments
#include "example/WidgetSix.h"
//  (WidgetSix includes WidgetAI/ALT/ASI/HI/TC/VSI/EADI/EHSI etc.)

#ifdef Q_OS_IOS
#undef Q_OS_MAC
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class SCREEN; }
QT_END_NAMESPACE

// Enable Android keep-awake helper (no-op on other platforms)
#define USE_KeepAwakeHelper

// --------------------------------------------------------------------------
// Platform-specific log/image directories
// --------------------------------------------------------------------------
#ifdef Q_OS_IOS
// iOS: user-visible Documents directory
#define LOG_DIR    QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
#define IMAGES_DIR QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
#elif defined(Q_OS_MAC)
// macOS: also use Documents
#define IMAGES_DIR QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
#define LOG_DIR    QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
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
     * @brief Initialize radio list with default values and save to file.
     * @return 0 on success, -1 on error.
     */
    int set_default_radio(void);

    /**
     * @brief Initialize airplane list with default values and save to file.
     * @return 0 on success, -1 on error.
     */
    int set_default_planes(void);

    /**
     * @brief Serialize calibration config (Accel/Gyro/Mag + IDs) to CONFIG file.
     * @param sensor 3x6 calibration matrix (Accel, Gyro, Mag rows).
     * @return 0 on success, -1 on error.
     */
    int set_default_config(const Matrix3x6 &sensor);

    /**
     * @brief Load calibration config from CONFIG file.
     * @param sensor [out] 3x6 calibration matrix.
     * @return 0 on success, negative on error (e.g. missing entries).
     */
    int get_default_config(Matrix3x6 &sensor);

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
     * @brief Refresh available camera list.
     */
    void updateCameras();

    /**
     * @brief Stop and hide camera preview.
     */
    void hideCamera();

    /**
     * @brief Select active camera and wire it into viewfinder / capture.
     */
    void setCamera(const QCameraDevice &cameraDevice);

    /**
     * @brief Called when location permission request resolves.
     */
    void permissionUpdated(const QPermission &permission);

    /**
     * @brief Helper to set a pixmap icon on a QPushButton.
     */
    void setButtonIcon(QString iconPath, QPushButton *button);

    /**
     * @brief Compute initial course (bearing) from (lat1,lon1) to (lat2,lon2).
     * @return bearing in degrees (0 = North, 90 = East).
     */
    double getBearing(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Calibration helper: update gravity estimate from accelerometer.
     */
    void AccelerometerRead();

    /**
     * @brief Compute and apply QNH to match baro altitude with GPS altitude.
     * @return QNH in hPa.
     */
    double setQNH();

    /**
     * @brief Load startup logo image and configure "fly home" button.
     */
    void showImage();

    /// Optional iOS splash screen shown at startup.
    QSplashScreen *splash = nullptr;

    // ------------------------------------------------------------------
    // Attitude / navigation state (public so instruments can be read easily)
    // ------------------------------------------------------------------

    /// Current attitude estimate [roll, pitch, yaw] in degrees.
    Vector3d m_attitude;

    /// Blended roll estimate (IMU + GPS turn-rate).
    double roll_blended = 0.0;
    bool   roll_blended_ok = false;

    /// Optional rotation matrix (not heavily used yet).
    Matrix3x3 rotationMatrix;

    /// Screen orientation (portrait/landscape).
    Qt::ScreenOrientation ScreenMode;

    /// Transponder code being set (digits 0..7).
    int next[4]   = {7, 0, 0, 0};

    /// Transponder active code.
    int current[4]= {8, 8, 8, 8};

    /// Current transponder mode (0..3).
    int mode = 0;

    /// IMU mounting orientation (tilt) used for compensation.
    Vector3d m_install;

    /// Instantaneous altitude change (delta) for vario calculation.
    double m_vario         = 0.0;

    /// GPS position at takeoff.
    double takeoff_latitude  = 0.0;
    double takeoff_longitude = 0.0;
    double takeoff_altitude  = 0.0;

    /// Transponder-reported altitude (feet or meters depending on mode).
    double m_tansALT = 0.0;

    /// True if an external IMU (via MyTcpSocket) is used instead of phone sensors.
    bool m_use_imu = false;

    /// Ground speed [km/h].
 //   double m_speed  = 0.0;

    /// Heading used by instruments (deg).
    double m_head   = 9999.0;

    /// Ambient temperature [°C].
    qreal  m_temp   = 9999.0;

    /// Roll angle derived from GPS turn rate (rad).
    double m_roll_angle   = 0.0;

    /// Net acceleration magnitude.
    double m_total_accel  = 0.0;

    /// Vertical speed [ft/min] filtered.
    double m_var_speed    = 0.0;

    /// Misc time variable (seconds).
    double m_ms           = 0.0;

    /// Radar depth (number of scan "columns").
    int Radar_depth  = 120;
    /// Radar height scaling.
    int Radar_Height = 120;

    /// Used during startup while BT scanning / IMU connection is in progress.
    bool m_bluetoothrunning = false;

    /// Takeoff and landing timestamps.
    QDateTime m_takeoffTime;
    QDateTime m_landedTime;

    /// Main hardware IO handler (transponder / radar / INS / MQTT).
    MyTcpSocket *mysocket = nullptr;

    // ------------------------------------------------------------------
    // Page indices in stacked widget
    // ------------------------------------------------------------------
    #define P_TRANSPONDER       0
    #define P_IMU               1
    #define P_FLIGT_INSTRUMENT  2
    #define P_GLASS_COCPIT      3
    #define P_RADAR             4
    #define P_RADIO_LIST        5
    #define P_AUTOPILOT         6
    #define P_CONFIG            7
    #define P_CAMERA            8

    /// Current index of stacked widget.
    int currentIndex = 0;

    /**
     * @brief C callback from MyTcpSocket when IMU presence is known.
     *
     * @param parent  Pointer back to MainWindow instance.
     * @param use_imu True if an external IMU is available.
     */
    static void setIMU(void *parent, bool use_imu);

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

    // ------------------------------------------------------------------
    // Instrument widgets (Qt analog style)
    // ------------------------------------------------------------------
    WidgetAI   *_widgetAI   = nullptr;
    WidgetTC   *_widgetTC   = nullptr;
    WidgetALT  *_widgetALT  = nullptr;
    WidgetASI  *_widgetASI  = nullptr;
    WidgetVSI  *_widgetVSI  = nullptr;
    WidgetHI   *_widgetHI   = nullptr;
    WidgetEADI *_widgetEADI = nullptr;
    WidgetEHSI *_widgetEHSI = nullptr;

    // Convenience getters
    WidgetAI   *getAI   () { return _widgetAI; }
    WidgetTC   *getTC   () { return _widgetTC; }
    WidgetALT  *getALT  () { return _widgetALT; }
    WidgetASI  *getASI  () { return _widgetASI; }
    WidgetVSI  *getVSI  () { return _widgetVSI; }
    WidgetHI   *getHI   () { return _widgetHI; }
    WidgetEADI *getEADI () { return _widgetEADI; }
    WidgetEHSI *getEHJSI() { return _widgetEHSI; }

private:
    // ------------------------------------------------------------------
    // Camera / media
    // ------------------------------------------------------------------
    QActionGroup        *videoDevicesGroup = nullptr;
    QMediaDevices        m_devices;
    QScopedPointer<QCamera> m_camera;
    QMediaCaptureSession m_captureSession;
    QMediaRecorder      *m_recorder     = nullptr;
    QCameraDevice       *m_cameraDevic  = nullptr;
    QImageCapture       *m_capture      = nullptr;

    // ------------------------------------------------------------------
    // Timers
    // ------------------------------------------------------------------
    bool    alt_receiced    = false;
    QTimer *m_Clock         = nullptr;  ///< 1Hz system clock.
    QTimer *timerAlt        = nullptr;  ///< Transponder alt check.
    QTimer *timerPing       = nullptr;  ///< Ping timeout.
    QTimer *timerActive     = nullptr;  ///< Activity timeout.
    QTimer *timerpaint      = nullptr;  ///< (reserved).
    QTimer *timertakePicture= nullptr;  ///< Periodic picture capture.
    QTimer *m_IMU           = nullptr;  ///< EKF update timer.
    QTimer *m_Display       = nullptr;  ///< Instrument redraw timer.

    // ------------------------------------------------------------------
    // EKF and attitude filter
    // ------------------------------------------------------------------
    ekfNavINS ekf;

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

    QOrientationSensor  *m_orientation_sensor  = nullptr;
    QOrientationReading *m_orientation_reader  = nullptr;

    QRotationSensor     *m_rotation_sensor  = nullptr;
    QRotationReading    *m_rotation_reader  = nullptr;

    QCompass            *m_compass_sensor   = nullptr;
    QCompassReading     *m_compass_reader   = nullptr;

    QAccelerometer      *m_accel_sensor     = nullptr;
    QAccelerometerReading *m_accel_reader   = nullptr;
    double               m_acc_Y_calib      = 0.0;

    QGyroscope          *m_gyro_sensor      = nullptr;
    QGyroscopeReading   *m_gyro_reader      = nullptr;

    QMagnetometer       *m_mag_sensor       = nullptr;
    QMagnetometerReading *m_mag_reader      = nullptr;

    QAmbientTemperatureSensor  *m_temp_sensor  = nullptr;
    QAmbientTemperatureReading *m_temp_reader  = nullptr;

    QGeoPositionInfoSource     *m_geoPositionInfo = nullptr;

    // ------------------------------------------------------------------
    // Misc state used in EKF / attitude blend
    // ------------------------------------------------------------------
    qreal  m_offset        = 0.0; ///< Pitch offset at calibration.
    double heading_offset  = 0.0; ///< Manual heading zeroing offset.
    bool   m_geopos        = false;
    double m_head_dir      = 0.0;
    double m_dt            = 0.0; ///< Time step [s] between EKF updates.
    Vector3d m_accel_body;        ///< Accel in body frame.

    double a_pitch         = 0.0;
    double a_roll          = 0.0;
    double a_yaw           = 0.0;
    double m_pitch         = 0.0;
    double m_roll          = 0.0;
    double m_yaw           = 0.0;

    QTimer *timerTRANS = nullptr; ///< Transponder polling timer.

    /// Calibration message box during IMU calibration.
    NoButtonMessageBox *m_msgBoxCalibrating = nullptr;

#if defined(Q_OS_ANDROID) && defined(USE_KeepAwakeHelper)
    /// Prevent Android device from sleeping during operation.
    KeepAwakeHelper *helper = new KeepAwakeHelper();
#endif

    /// Signal emitted on quit/OK (used with dialogs).
    void accepted();

    void calcPosition(double vel_D);

    void afterPressureReadingChanged();


private slots:
    // ------------------------------------------------------------------
    // Simple UI slots / transponder keypad / exit etc.
    // ------------------------------------------------------------------
    void setVal();

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

    // Misc controls
    void on_reconnect_now_clicked();
    void on_pushButton_20_clicked();
    void on_reset_heading_clicked();
    void on_select_transponder_page_3_clicked();
    void on_select_from_4_to_5_clicked();

    // Status / timers
    void doCheck();
    void reset_ping();
    void active_ping();
    void doClock();

    // GPS / position updates
    void positionUpdated(QGeoPositionInfo geoPositionInfo);

    // Sensor change notifications
    void onRotationReadingChanged();
    void onReadingChanged();
    void onPressureReadingChanged();
    void onCompassReadingChanged();
    void onAccelerometerReadingChanged();
    void onGyroReadingChanged();
    void onMagReadingChanged();
    void onOrientationReadingChanged();
    void onTempReadingChanged();

    // Page navigation (stackedWidget)
    void on_select_gyro_page_clicked();
    void on_select_gyro_page2_clicked();
    void on_select_camera_from_transponder_clicked();
    void on_select_dumy_page2_clicked();
    void on_select_transponder_page_clicked();
    void on_select_transponder_page2_clicked();
    void on_imu_reset_clicked();
    void on_timer_start_clicked();
    void on_textEdit_1_textChanged();
    void on_textEdit_2_textChanged();
    void on_pushButton_15_clicked();
    void on_select_transponder_page2_2_clicked();
    void on_select_gyro_page2_2_clicked();
    void on_select_transponder_page2_3_clicked();
    void on_select_gyro_page2_3_clicked();
    void on_pushButton_21_clicked();
    void on_pushButton_22_clicked();
    void on_select_dumy_page2_2_clicked();
    void on_select_transponder_page_2_clicked();
    void on_select_page2_map_clicked();
    void on_select_transponder_page_camera_clicked();
    void on_use_gps_in_attitude_clicked();
    void on_select_transponder_page_4_clicked();
    void on_select_from_5_to_6_clicked();
    void on_use_built_inn_barometer_clicked();

    void on_pushButton_23_clicked();
    void on_exit_2_clicked();
    void on_use_hw_clicked();
    void on_reset_att_clicked();
    void on_fly_home_clicked();

    void on_reset_altitude_3_clicked();
    void on_reset_altitude_2_clicked();
    void on_reset_heading_2_clicked();
    void on_use_ins_only_clicked();

    // Camera
    void takePicture();
    void on_dial_valueChanged(int value);
    void on_dial_2_valueChanged(int value);

    // EKF main loop
    void EKF();

    /**
     * @brief C-style RX callback from MyTcpSocket for transponder data.
     *
     * @param parent Pointer back to MainWindow instance.
     * @param data   Raw ASCII payload.
     * @param lenght Length of payload in bytes.
     */
    static void getVal(void *parent, const char *data, uint32_t lenght);

public:
    /// Which screen (monitor) index we are using.
    int screen_index = 0;

    /// Qt Designer-generated UI class instance.
    /// NOTE: constructed in a custom way; left unchanged for compatibility.
    Ui::SCREEN *ui = (Ui::SCREEN *) &(*new (Ui::SCREEN));
};

#endif // MAINWINDOW_H
