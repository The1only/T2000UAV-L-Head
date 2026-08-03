/**
 * @file mainwindow.cpp
 * @brief Implementation of MainWindow.
 *
 * Contains the implementation details for the MainWindow class.
 */

#include <QtCore/QLoggingCategory>
#include <QQmlContext>
#include <QGuiApplication>
#include <QColorDialog>
#include <QNetworkInterface>
#include <QTimer>
#include <cstdio>
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QDir>
#include <QStandardPaths>
#include <QQuickWidget>
#include <QQmlProperty>
#include <QPermission>
#include <QActionGroup>
#include <QVideoWidget>
#include <QCameraDevice>
#include <QPixmap>
#include <QMediaRecorder>
#include <QImageCapture>
#include <QMediaFormat>
#include <QMediaPlayer>
#include <QOrientationSensor>
#include <QImageCapture>
#include <QList>
#include <QSplashScreen>
#include <QtMath>  // for qDegreesToRadians, qRadiansToDegrees
#include <deque>
#include <QThread>

//***C++11 Style:***
#include <chrono>

#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>

#include "mainwindow.h"
//#include "mytcpsocket.h"
#include "gpx_parse.h"

#include "wit_c_sdk.h"
#include "geoid_helper.h"

using namespace std;
using namespace std::chrono;

// ----------------------------------------------
// ----------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    _widgetAI   ( Q_NULLPTR ),
    _widgetTC   ( Q_NULLPTR ),
    _widgetALT  ( Q_NULLPTR ),
    _widgetASI  ( Q_NULLPTR ),
    _widgetVSI  ( Q_NULLPTR ),
    _widgetHI   ( Q_NULLPTR ),
    _widgetEADI ( Q_NULLPTR ),
    _widgetEHSI ( Q_NULLPTR )
{

/*
    QSize design(800, 600);
    QSize screen = QGuiApplication::primaryScreen()->size();

    float scale = qMin(
        screen.width() / float(design.width()),
        screen.height() / float(design.height())
        );

    this->resize(design * scale);
    this->setFixedSize(this->size());
*/

#if defined(Q_OS_IOS) || defined(Q_OS_MACOS)
    // Documents directory (user-visible, backed up)
    QString documentsPath = LOG_DIR;
    QDir dir(documentsPath);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
#endif

    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(currentIndex);


// The splash screen does not make sense on a PC... but it works if you need it...
//#if defined(Q_OS_IOS) || defined(Q_OS_MACOS)
#if defined(Q_OS_IOS)
    QPixmap splashPixmap(":/images/splash.png");  // Or use a file path
    splash = new QSplashScreen(splashPixmap);
    splash->autoFillBackground();
//    splash->showMessage("Initializing Flight IMU...", Qt::AlignTop | Qt::AlignCenter, Qt::black);
    splash->show();
#endif

    // --------------------------------
    // Setup the Radio List...
    double temp_g=0.0;
    qDebug() << "Reading files..." << QString(LOG_DIR) << QString(RADIO);
    QString blob1;
    {
        QFile *l_file = new QFile(QString(LOG_DIR)+ QString(RADIO));
        if (l_file->open(QIODevice::ReadOnly))
        {
            blob1 = l_file->readAll();
            l_file->close();
        }
        else{
            set_default_radio();
        }
    }
    // Setup the Airplane List...
    QString blob2;
    {
        QFile *l_file = new QFile(QString(LOG_DIR)+ QString(AIRPLANE));
        if (l_file->open(QIODevice::ReadOnly))
        {
            blob2 = l_file->readAll();
            l_file->close();
        }
        else{
            set_default_planes();
        }
    }

    ui->textEdit->insertPlainText(blob1);
    ui->textEdit_2->insertPlainText(blob2);

    // --------------------------

   // ui->quickWidget->setSource(QUrl("qrc:/places_map.qml"));
   // ui->quickWidget->rootObject()->setProperty("zoomLevel", 35); // 18);

    ui->plainTextEdit->document()->setMaximumBlockCount(50);
    ui->lcdNumber->display(QString::number(this->current[0]*1000+this->current[1]*100+this->current[2]*10+this->current[3]).rightJustified(4, '0'));
    ui->lcdNumber_2->display(QString::number(this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]).rightJustified(4, '0'));
    ui->plainTextEdit->appendPlainText("Glasscockpit 200-UAV v1.02a");
    ui->listView->appendPlainText("Glasscockpit 200-UAV v1.02a");
    ui->listView->raise();
    ui->listView->setHidden(true);

    // This code must be rewritten as it is depending on timeing and speed.
    // The serial ports shuld be a parameter to the constructor...
    // The m_calibrate shuld be set in a different manner...
    // Remember that the MyTcpSocket spawns a slower process only...
    this->mysocket = new MyTcpSocket(this, ui->plainTextEdit, &this->setIMU);

    QThread::msleep(1000);

    // Setup the Config List...
    Matrix3x6 config;
    memset(&config,0,sizeof(Matrix3x6));
    if(get_default_config(config) != 0)
    {
        config[0][2] = (double)Gfix;
        set_default_config(config);
    }

    qDebug() << "AccValue" << config[0][2];
    temp_g = config[0][2];
    ekf.ekf->Gval       = temp_g;
    ekf.ekf_quart->Gval = temp_g;
    ekf.Gval            = temp_g;
    this->mysocket->G   = temp_g;
    qDebug() << "Saved G value is: " << temp_g;

    // Whenever the location data source signals that the current
    // position is updated, the positionUpdated function is called.
    this->m_geoPositionInfo = QGeoPositionInfoSource::createDefaultSource(this);
    if (this->m_geoPositionInfo)
    {
        connect(m_geoPositionInfo,SIGNAL(positionUpdated(QGeoPositionInfo)),this,SLOT(positionUpdated(QGeoPositionInfo)));
        // Start listening for position updates
        m_geoPositionInfo->setUpdateInterval(1000);
        m_geoPositionInfo->startUpdates();
    }
    // ------------------------------

    qDebug() << "  setmode standby ";
    setmode(1);

    qDebug() << "  m_timer  ";
    m_timer.start();

    qDebug() << "  timerPing  ";
    timerPing = new QTimer(this);
    timerPing->setSingleShot(true);
    connect(timerPing, SIGNAL(timeout()), this, SLOT(reset_ping()));

    qDebug() << "  timerActive ";
    timerActive = new QTimer(this);
    timerActive->setSingleShot(true);
    connect(timerActive, SIGNAL(timeout()), this, SLOT(active_ping()));

    qDebug() << "  timerPictureActive ";
    timertakePicture = new QTimer(this);
    timertakePicture->setSingleShot(false);
    connect(timertakePicture, SIGNAL(timeout()), this, SLOT(takePicture()));

    qDebug() << "  timerAlt  ";
    timerAlt = new QTimer(this);
    timerAlt->setSingleShot(false);
    connect(timerAlt, SIGNAL(timeout()), this, SLOT(doCheck()));
    timerAlt->start(5000);

    QString data = "System booted at: "+QDateTime::currentDateTime().toString();
    ui->listView->appendPlainText(data);

    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
    if( l_file->open(QIODevice::ReadWrite | QIODevice::Append ))
    {
        l_file->write(data.toLocal8Bit()+"\n");
        l_file->close();
    }
    else{
        qDebug() << "Log file error...  ";
    }
    // try to actually initialize camera & mic
#ifndef TRANSPONDER_ONLY
    init();
#endif

    _widgetAI  = ui->widgetai;
    _widgetAI->reinit();
    _widgetTC  = ui->widgetTC;
    _widgetTC->reinit();
    _widgetALT  = ui->widgetALT;
    _widgetALT->reinit();
    _widgetASI  = ui->widgetASI;
    _widgetASI->reinit(0);
    _widgetVSI  = ui->widgetVSI;
    _widgetVSI->reinit();
    _widgetHI  = ui->widgetHI;
    _widgetHI->reinit();
    _widgetEADI  = ui->widgetEADI;
    _widgetEADI->reinit();
    _widgetEHSI  = ui->widgetEHSI;
    _widgetEHSI->reinit();

    _widgetHI->setHeading(0);
    _widgetHI->redraw();

    _widgetVSI->setClimbRate(0);
    _widgetVSI->redraw();

    _widgetASI->setAirspeed(0);
    _widgetASI->redraw();

    _widgetALT->setAltitude(0);
    _widgetALT->redraw();

    _widgetTC->setTurnRate(0);
    _widgetTC->setSlipSkid(0);
    _widgetTC->redraw();

    _widgetAI->setRoll ( 0 );
    _widgetAI->setPitch( 0 );
    _widgetAI->redraw();


    _widgetEADI->setFltMode( qfi_EADI::FltMode::Off);
    _widgetEADI->setSpdMode( qfi_EADI::SpdMode::Off);
    _widgetEADI->setLNAV( qfi_EADI::LNAV::Off);
    _widgetEADI->setVNAV( qfi_EADI::VNAV::Off);
    _widgetEADI->setRoll( 0.0 );
    _widgetEADI->setPitch( 0.0 );
    _widgetEADI->setFPM( 0.0, 0.0 );
    _widgetEADI->setSlipSkid( 0.0 );
    _widgetEADI->setTurnRate( 0.0 );
    _widgetEADI->setDots( 0.0,0.0,false,false);
    _widgetEADI->setFD( 0.0,0.0,true);
    _widgetEADI->setStall( false );
    _widgetEADI->setAltitude( 0.0 );
    _widgetEADI->setPressure( 1024.13, qfi_EADI::PressureMode::MB );
    _widgetEADI->setAirspeed( 0.0 );
    _widgetEADI->setMachNo( 0.0 );
    _widgetEADI->setHeading( 0.0 );
    _widgetEADI->setClimbRate( 0.0 );
    _widgetEADI->setAirspeedSel( 0.0 );
    _widgetEADI->setAltitudeSel(0 );
    _widgetEADI->setHeadingSel( 0.0 );
    _widgetEADI->setVfe( 0.0 );
    _widgetEADI->setVne( 0.0 );
    _widgetEADI->redraw();

    _widgetEHSI->setHeading( 45);
    _widgetEHSI->setCourse( 270 );
    _widgetEHSI->setBearing( 280, true );
    _widgetEHSI->setDeviation( 4.0, qfi_EHSI::CDI::FROM);
    _widgetEHSI->setDistance( 200, true );
    _widgetEHSI->setHeadingSel( 50 );
    _widgetEHSI->setHeading(0);
    _widgetEHSI->redraw();

    // Save the current index page...
    currentIndex = ui->stackedWidget->currentIndex();

    showImage();


    m_msgBox = new NoButtonMessageBox(tr("Please wait for the system to boot!"));
 //   m_msgBox->show();

    qDebug() << "  Booting step two...  " << currentIndex << " Index";

    //Bøverbru.fpl
    QString flightplan = IMAGES_DIR+QString("/Boverbru.gpx");
    GpxParser parser;
    static QList<TrackPoint> points;
    if (parser.parseFile(flightplan))
    {
        points = parser.getTrackPoints();
        // IF we found some waypoints...
        while(!points.isEmpty())
        {
            TrackPoint pt = points.takeFirst();
            //qDebug() << pt.elevation << pt.latitude << pt.longitude << pt.name;
        }
    }

#ifdef TRANSPONDER_ONLY
    ui->select_down->hide();
    ui->select_up->hide();
    ui->select_down->setDisabled(true);
    ui->select_up->setDisabled(true);
#endif

    // Start the clock....
    qDebug() << "  Transponder update...  ";
    m_Transponder = new QTimer(this);
    m_Transponder->setSingleShot(false);
    connect(m_Transponder, SIGNAL(timeout()), this, SLOT(getTransponderVal()));
    m_Transponder->start(250);

    // Start the clock....
    qDebug() << "  doClock  ";
    m_Clock = new QTimer(this);
    m_Clock->setSingleShot(false);
    connect(m_Clock, SIGNAL(timeout()), this, SLOT(doClock()));
    m_Clock->start(1000);

}

MainWindow::~MainWindow()
{
    qDebug() << "Exiting...";
    qApp->closeAllWindows();
}


void MainWindow::showImage()
{
    /*
    auto *view = ui->graphicsView_3;                 // <-- use one view consistently
    QGraphicsScene *scene = view->scene();
    if (!scene) {
        scene = new QGraphicsScene(view);
        view->setScene(scene);
    }

    QPixmap pix(":/images/test1.png");                      // from resources
    if (pix.isNull()) {
        qWarning() << "Failed to load :/images/test1.png; check .qrc path";
        return;
    }

    scene->clear();
    QGraphicsPixmapItem *item = scene->addPixmap(pix);
    item->setTransformationMode(Qt::SmoothTransformation);
    scene->setSceneRect(item->boundingRect());
    view->fitInView(item, Qt::IgnoreAspectRatio);

    view->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    // Fit now; if the view isn't laid out yet, refit on next event loop tick
    view->fitInView(item, Qt::IgnoreAspectRatio);
    QTimer::singleShot(0, view, [view, item]{
        view->fitInView(item, Qt::IgnoreAspectRatio);
    });

    ui->fly_home->setText("");
    ui->fly_home->setIcon(QIcon(":/images/engage.png"));
    ui->fly_home->setFlat(true); // optional
    QSize f = ui->fly_home->size();
    ui->fly_home->setIconSize(f);
*/
    //-----------------------------------------------
    // To be able to set the opacity, we can NOT do this in the UI editor...
    auto *view = ui->graphicsView_2;                 // <-- use one view consistently
    QGraphicsScene *m_graphScen = view->scene();
    if (!m_graphScen) {
        m_graphScen = new QGraphicsScene(view);
        view->setScene(m_graphScen);
    }

    QPixmap pix2(":/images/radar_3.png");                      // from resources
    if (pix2.isNull()) {
        qWarning() << "Failed to load :/radar_3.png; check .qrc path";
        return;
    }

    // Write where the log files will be...
    QString logDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(logDir);
    ui->plainTextEdit->appendPlainText(logDir);

    m_graphScen->clear();
    QGraphicsPixmapItem *item = m_graphScen->addPixmap(pix2);
    item->setTransformationMode(Qt::SmoothTransformation);
    item->setOpacity(1.0);  //
    m_graphScen->setSceneRect(item->boundingRect());
    view->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    view->fitInView(item, Qt::IgnoreAspectRatio);
    QTimer::singleShot(0, view, [view, item]{
        view->fitInView(item, Qt::IgnoreAspectRatio);
    });

    //-----------------------------------------------
}

//***************************************************************************************************************//
void MainWindow::setButtonIcon(QString iconPath, QPushButton* button)
{
    QString str("Test");
    qApp->applicationDirPath().append(iconPath);
    QPixmap pixmap(str);
    QIcon buttonIcon(pixmap);
    button->setIcon(buttonIcon);
    button->setIconSize(pixmap.rect().size());
}

// Call back funtion from the sensor handler...
void MainWindow::setIMU(void *parent, bool use_imu)
{
#ifndef TRANSPONDER_ONLY

    static bool allready_run = false;
    MainWindow* local = (MainWindow*)parent;
    QList<QSensor*> mySensorList;

    local->m_use_imu = use_imu;
    if(use_imu) qDebug() << "Found IMU/INS sensor...";
    else qDebug() << "NOT Found any IMU/INS sensors...";

    if(allready_run == false)
    {
        allready_run = true;
        for(int i=0; i < 1000; i++){
            QCoreApplication::processEvents();
            QThread::msleep(1);
        }

        // If we got external air preassure as part of the IMU or standalone...
        if(local->mysocket->m_pressure_raw > 1.0 )
        {
            qDebug() << "Found a external sensor QPressureSensor";
            local->ui->radioButton_3->setChecked(true);
/*
            // If we got both a preassure sensor and the Transponder ...
            if( local->mysocket->Transponderstat == true)
            {
                local->mysocket->readyWrite((char*)"\x02" "d=s" "\x03");
                local->mysocket->TransponderstatWithBarometer = true;

                QString x = local->ui->use_built_inn_barometer->styleSheet();
                x.replace(QString("1 #080"), QString("1 #800"));
                local->ui->use_built_inn_barometer->setText("Use Built In\nbarometer");
                local->ui->use_built_inn_barometer->setStyleSheet(x);
                local->ui->use_built_inn_barometer->update();
            }
*/
        }

        for (const QByteArray &type : QSensor::sensorTypes())
        {
            qDebug() << "Found a sensor type:" << type;

            for (const QByteArray &identifier : QSensor::sensorsForType(type))
            {
                qDebug() << "    " << "Found a sensor of that type:" << identifier;
                QSensor* sensor = new QSensor(type, local);
                sensor->setIdentifier(identifier);
                mySensorList.append(sensor);
            }

            // If we does NOT have an external sensor...
            if(!local->ui->radioButton_3->isChecked()){
                if(!strncmp(type,"QPressureSensor",strlen("QPressureSensor")))
                {
                    local->m_pressure_sensor = new QPressureSensor();
                    connect(local->m_pressure_sensor, SIGNAL(readingChanged()), local, SLOT(onPressureReadingChanged()));
                    local->m_pressure_sensor->start();
                    local->m_pressure_sensor->setDataRate(4);
                    qDebug() << "Found a sensor QPressureSensor";
                    local->ui->radioButton_3->setChecked(true);

                    // If we got both a preassure sensor and the Transponder ...
                    /*
                    if( local->mysocket->Transponderstat == true)
                    {
                        local->mysocket->readyWrite((char*)"\x02" "d=s" "\x03");
                        local->mysocket->TransponderstatWithBarometer = true;

                        QString x = local->ui->use_built_inn_barometer->styleSheet();
                        x.replace(QString("1 #080"), QString("1 #800"));
                        local->ui->use_built_inn_barometer->setText("Use Built In\nbarometer");
                        local->ui->use_built_inn_barometer->setStyleSheet(x);
                        local->ui->use_built_inn_barometer->update();
                    }
*/
                }
            }

            if(!strncmp(type,"QOrientationSensor",strlen("QOrientationSensor")))
            {
                local->m_orientation_sensor = new QOrientationSensor();
                local->m_orientation_sensor->setDataRate(1);
                connect(local->m_orientation_sensor, SIGNAL(readingChanged()), local, SLOT(onOrientationReadingChanged()));
                local->m_orientation_sensor->start();
                qDebug() << "Found a sensor QRotationSensor";
            }

            if(!strncmp(type,"QAmbientTemperatureSensor",strlen("QAmbientTemperatureSensor")))
            {
                local->m_temp_sensor = new QAmbientTemperatureSensor();
                local->m_temp_sensor->setDataRate(1);
                connect(local->m_temp_sensor, SIGNAL(readingChanged()), local, SLOT(onTempReadingChanged()));
                local->m_temp_sensor->start();
                qDebug() << "Found a sensor QAmbientTemperatureReading";
            }

            // If we do not have an external IMU...
            if(local->m_use_imu  == false)
            {
                if(!strncmp(type,"QRotationSensor",strlen("QRotationSensor")))
                {
                    local->m_rotation_sensor = new QRotationSensor();
                    local->m_rotation_sensor->setDataRate(50);
                    connect(local->m_rotation_sensor, SIGNAL(readingChanged()), local, SLOT(onRotationReadingChanged()));
                    local->m_rotation_sensor->start();
                    qDebug() << "Found a sensor QRotationSensor";
                }

                if(!strncmp(type,"QCompass",strlen("QCompass")))
                {
                    local->m_compass_sensor = new QCompass();
                    connect(local->m_compass_sensor, SIGNAL(readingChanged()), local, SLOT(onCompassReadingChanged()));
                    local->m_compass_sensor->start();
                    local->m_compass_sensor->setDataRate(50);
                    qDebug() << "Found a sensor QCompass";
                }

                if(!strncmp(type,"QAccelerometer",strlen("QAccelerometer")))
                {
                    local->m_accel_sensor = new QAccelerometer();
                    local->m_accel_sensor->setDataRate(50);
                    connect(local->m_accel_sensor, SIGNAL(readingChanged()), local, SLOT(onAccelerometerReadingChanged()));
                    local->m_accel_sensor->start();
                    qDebug() << "Found a sensor QAccelerometerReading";
                }

                if(!strncmp(type,"QGyroscope",strlen("QGyroscope")))
                {
                    local->m_gyro_sensor = new QGyroscope();
                    local->m_gyro_sensor->setDataRate(50);
                    connect(local->m_gyro_sensor, SIGNAL(readingChanged()), local, SLOT(onGyroReadingChanged()));
                    local->m_gyro_sensor->start();
                    qDebug() << "Found a sensor QGyroscopeReading";
                }

                if(!strncmp(type,"QMagnetometer",strlen("QMagnetometer")))
                {
                    local->m_mag_sensor = new QMagnetometer();
                    local->m_mag_sensor->setDataRate(50);
                    connect(local->m_mag_sensor, SIGNAL(readingChanged()), local, SLOT(onMagReadingChanged()));
                    local->m_mag_sensor->start();
                    qDebug() << "Found a sensor QMagnetometerReading";
                }
            }
            /*
            else{
                if(!strncmp(type,"QRotationSensor",strlen("QRotationSensor")))
                {
                    local->m_rotation_sensor = new QRotationSensor();
                    local->m_rotation_sensor->setDataRate(4);
                    connect(local->m_rotation_sensor, SIGNAL(readingChanged()), local, SLOT(onRotationReadingChanged()));
                    local->m_rotation_sensor->start();
                    qDebug() << "Found a sensor QRotationSensor";
                }
            }
            */

        }
        // If we do not have an external IMU...
        if(use_imu == false)
        {
            QString data = "IMU NOT found...";
            local->ui->listView->appendPlainText(data);
            QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
            if( l_file->open(QIODevice::ReadWrite | QIODevice::Append )){
                l_file->write(data.toLocal8Bit()+"\n");
                l_file->close();
            }
        }
        else{
            QString data = "IMU found and connected...";
            local->ui->listView->appendPlainText(data);
            local->ekf.m_use_gpt = 0;
            QString x = local->ui->reset_att->styleSheet();
            x.replace(QString("1 ##888"), QString("1 #f00"));
            x.replace(QString("1 #0F0"),  QString("1 #f00"));
            local->ui->reset_att->setStyleSheet(x);
            local->ui->reset_att->update();
            QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
            if( l_file->open(QIODevice::ReadWrite | QIODevice::Append ))
            {
                l_file->write(data.toLocal8Bit()+"\n");
                l_file->close();
            }
            local->m_calibrate = 100;
        }

        qDebug() << mySensorList;

        if(local->ui->radioButton_3->isChecked() == false)
        {
            local->ui->radioButton_3->setCheckable(false);
            local->ui->dial->hide();
            local->ui->dial_2->hide();
            local->ui->doubleSpinBox->hide();
            local->ui->doubleSpinBox_2->hide();

            QString x = local->ui->label_18->styleSheet();
            x.replace(QString("135"), QString("80"));
            local->ui->label_18->setStyleSheet(x);
            local->ui->label_18->update();
        }

        local->m_Display = new QTimer(local);
        local->m_Display->setSingleShot(false);
        connect(local->m_Display, SIGNAL(timeout()), local, SLOT(onReadingChanged()));
        local->m_Display->start(100);

        local->m_IMU = new QTimer(local);
        local->m_IMU->setSingleShot(false);
        connect(local->m_IMU, SIGNAL(timeout()), local, SLOT(EKF()));
        local->m_IMU->start(20);
        local->m_dt = QDateTime::currentMSecsSinceEpoch();

    //    local->m_msgBox->hide();
    //    QCoreApplication::processEvents();
        local->m_msgBox->show();
    }
#endif
    QCoreApplication::processEvents();
}

void MainWindow::permissionUpdated(const QPermission &permission)
{
    if (permission.status() != Qt::PermissionStatus::Granted)
    {
        qDebug() << "Precise location permission denied";
        return;
    }
    auto locationPermission = permission.value<QLocationPermission>();
    if (!locationPermission || locationPermission->accuracy() != QLocationPermission::Precise)
    {
        qDebug() << "Precise location permission error";
        return;
    }
    qDebug() << "Precise location OK";
}

void MainWindow::init()
{
#if QT_CONFIG(permissions)
    // camera
    QCameraPermission cameraPermission;
    switch (qApp->checkPermission(cameraPermission))
    {
        case Qt::PermissionStatus::Undetermined:
            qApp->requestPermission(cameraPermission, this, &MainWindow::init);
            return;
        case Qt::PermissionStatus::Denied:
            qWarning("Camera permission is not granted!");
            return;
        case Qt::PermissionStatus::Granted:
            break;
    }

    QLocationPermission locationPermission;
    locationPermission.setAccuracy(QLocationPermission::Precise);
    qApp->requestPermission(locationPermission, this, &MainWindow::permissionUpdated);

#endif

    // Camera devices:
    videoDevicesGroup = new QActionGroup(this);
    videoDevicesGroup->setExclusive(true);
    connect(&m_devices, &QMediaDevices::videoInputsChanged, this, &MainWindow::updateCameras);
    const QRect *m_vsize = &ui->viewfinder->geometry();
    m_size = new QSize( m_vsize->width(),m_vsize->height());
    //    qDebug() << "TERJE:::::::" <<  m_vsize->height() << "    "  <<  m_vsize->width();
}


void MainWindow::updateCameras()
{
    const QList<QCameraDevice> availableCameras = QMediaDevices::videoInputs();
    for (const QCameraDevice &cameraDevice : availableCameras)
    {
        QAction *videoDeviceAction = new QAction(cameraDevice.description(), videoDevicesGroup);
        videoDeviceAction->setCheckable(true);
        videoDeviceAction->setData(QVariant::fromValue(cameraDevice));
        if (cameraDevice == QMediaDevices::defaultVideoInput())
            videoDeviceAction->setChecked(true);
    }
}

void MainWindow::setCamera(const QCameraDevice &cameraDevice)
{
    m_cameraDevic = new QCameraDevice(cameraDevice);
    m_camera.reset(new QCamera(*m_cameraDevic));
    m_captureSession.setCamera(m_camera.data());

    m_captureSession.setVideoOutput(ui->viewfinder);
    ui->viewfinder->resize(*m_size);
    ui->viewfinder->show();

    /*Initialize a QImageCapture instance*/
    m_capture = new QImageCapture;
    m_captureSession.setImageCapture(m_capture);

    QDir dir(IMAGES_DIR);
    if (!dir.exists()){
        dir.mkpath(".");
    }

    /*Initialize a QImageCapture instance*/
    connect(m_capture, &QImageCapture::imageCaptured,[this](int id, const QImage &preview)
        {
            qDebug() << "Image Captured...";
            ui->plainTextEdit->appendPlainText(QString("Image Captured:%1").arg(id));
            QDateTime date = QDateTime::currentDateTime();
            QString filename2 = IMAGES_DIR+QString("/")+date.toString("yyyy_dd_MM_hh_mm_ss").append(".jpg");
            QPixmap pixmap(QPixmap::fromImage(preview));
            QImage *imagex = new QImage(pixmap.toImage());
            imagex->save(filename2, "jpg", 100);
    /*
            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
    */
            /*
            QString filename1 = IMAGES_DIR+QString("/")+date.toString("yyyy_dd_MM_hh_mm_ss").append(".avi");
            imwrite("alpha2.png", frame, compression_params);
            VideoWriter video(filename1, CV_FOURCC('M','J','P','G'), 10, Size(qImageSingle.width(), qImageSingle.height()), true);
            for(int i=0; i<100; i++){
                video.write(frame); // Write frame to VideoWriter
            }
            */
        });

    m_camera->start();
}

void MainWindow::hideCamera()
{
    ui->viewfinder->resize(QSize(1,1));

    m_camera->stop();
    ui->viewfinder->hide();
    m_captureSession.disconnect();
    m_camera.reset();

}

void MainWindow::logTakeoff()
{
    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
    m_takeoffTime = QDateTime::currentDateTime();
    QString data = "Takeoff at: "+m_takeoffTime.toString();
    ui->listView->appendPlainText(data);

    if( l_file->open(QIODevice::ReadWrite | QIODevice::Append ))
    {
        l_file->write(data.toLocal8Bit()+"\n");
        l_file->close();
    }
}

void MainWindow::logLanded()
{ 
    m_landedTime = QDateTime::currentDateTime();
    QString data = "Landed at: "+m_landedTime.toString();
    QTime dif(0,0,0,0);
    QTime t=dif.addMSecs(m_landedTime.toMSecsSinceEpoch()-m_takeoffTime.toMSecsSinceEpoch());
    QString result=t.toString("hh:mm:ss.zzz");
    QString dtime = "Duration: "+result;
    ui->listView->appendPlainText(data);
    ui->listView->appendPlainText(dtime);

    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
    if( l_file->open(QIODevice::ReadWrite | QIODevice::Append ))
    {
        l_file->write(data.toLocal8Bit()+"\n");
        l_file->write(dtime.toLocal8Bit()+"\n");
        l_file->close();
    }
}

void MainWindow::doClock()
{
    static int tim = 0;

    if(this->mysocket != nullptr){
        if(!(++tim % 10)){
            static double lastTemp = -1;
            if(lastTemp != mysocket->Temp)
            {

            //    uint8_t major = (mysocket->VER  >> 8) & 0xFF;
            //    uint8_t minor = mysocket->VER  & 0xFF;
            //    printf("Firmware Version: %d.%d\n", major, minor);

//              ui->plainTextEdit_2->appendPlainText(
                this->ui->plainTextEdit_2->setPlainText(
                    QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss "));

//                ui->plainTextEdit_2->setPlainText(
//                    QString("IMU Ver: %1.%2\nTemp: %3").arg(major).arg(minor).arg( mysocket->Temp));
                lastTemp = mysocket->Temp;
            }
        }

        // If we are in the air...
        if(m_takeoff){
            ui->timeEdit_2->setTime(QTime::fromMSecsSinceStartOfDay(m_timer.elapsed()));
            // Have we landed...
            if(mysocket->m_speed <= 5.0)
            {
                m_takeoff = false;
                logLanded();
            }
        }
        else{
            /*
            double alt = this->mysocket->m_altitude;

            // If we are usimng GPS the forse it to be set...
            if(!this->ui->radioButton_2->isChecked()){
                if( this->mysocket->m_pressure_raw > 1.0 )
                {
                    alt = this->mysocket->m_preasure_alt;
                }
            }
*/
            // If more than 40Km/t we are taking off...
            if( mysocket->m_speed > 40.0 )
            {
                m_takeoff = true;

                this->takeoff_latitude  = this->mysocket->m_latitude;
                this->takeoff_longitude = this->mysocket->m_longitude;
                this->takeoff_altitude  = this->mysocket->m_altitude;

                logTakeoff();
            }
        }
    }

    ui->label_UTC->setText(QDateTime::currentDateTimeUtc().toString("hh:mm:ss"));
//      ui->label_UTC->setText(QDateTime::currentDateTime().toString("hh:mm:ss") );
    ui->timeEdit->setDateTime(QDateTime::currentDateTime());


// Calculate meters/s ...
#define varfilterlength 4

    static double vario = 0;
    double var;

    if( this->mysocket->m_pressure_raw > 1.0 )
    {
        m_vario = this->mysocket->m_preasure_alt - vario;
        vario = this->mysocket->m_preasure_alt;
        var=m_vario*60;  // Feet/m...
    }
    else{
        m_vario = this->mysocket->m_altitude - vario;
        vario = this->mysocket->m_altitude;
        var = m_vario *60;// Feet/m...
    }

    static double varfilter[varfilterlength]={0};
    double var_speed = 0;
    for(int x=0; x < varfilterlength-1;x++)
    {
        varfilter[x] = varfilter[x+1];
        var_speed+=varfilter[x];
    }
    varfilter[varfilterlength-1] = var;
    var_speed+=var;
    m_var_speed=var_speed/varfilterlength;

    if(simGPS == true){
        static QGeoPositionInfo x;
        positionUpdated(x);
    }
    // If we got an IMU with preassure...
    if(mysocket->m_pressure_raw > 1.0 ){
        if(this->mysocket->Altimeter_data.temperature != 0.0){
            m_temp = this->mysocket->Altimeter_data.temperature;
        }
        afterPressureReadingChanged();
    }

    // If we got an IMU with GPS...
    if(mysocket->m_altitude > 0){
        calcPosition(mysocket->Donwn_Speed);
    }
}

void MainWindow::onGyroReadingChanged()
{
/*
    static steady_clock::time_point clock_begin = steady_clock::now();
    steady_clock::time_point clock_end = steady_clock::now();
    steady_clock::duration time_span = clock_end - clock_begin;
    qDebug() << double(time_span.count()) / 1000000000L;
    clock_begin=clock_end;
*/
    if(!mysocket->m_has_MQTT_gyro)
    {
        m_gyro_reader = m_gyro_sensor->reading();
        mysocket->AsX = m_gyro_reader->x();
        mysocket->AsY = m_gyro_reader->y();
        mysocket->AsZ = m_gyro_reader->z();
    }
}

void MainWindow::onMagReadingChanged()
{
    m_mag_reader = m_mag_sensor->reading();
    mysocket->HX = m_mag_reader->x();
    mysocket->HY = m_mag_reader->y();
    mysocket->HZ = m_mag_reader->z();
}

void MainWindow::onAccelerometerReadingChanged()
{
    if(!mysocket->m_has_MQTT_accel)
    {
        m_accel_reader = m_accel_sensor->reading();
        mysocket->AccX = m_accel_reader->x();  // YAW
        mysocket->AccY = -m_accel_reader->y(); // ROLL
        mysocket->AccZ = -m_accel_reader->z(); // PITCH
    }
}

void MainWindow::onTempReadingChanged()
{
    m_temp_reader = m_temp_sensor->reading();
    m_temp = m_temp_reader->temperature();

}

void MainWindow::onCompassReadingChanged()
{
    if(!mysocket->m_has_MQTT_heading)
    {
        m_compass_reader = m_compass_sensor->reading();
        //    m_heading = m_compass_reader->azimuth();
    }
}

void MainWindow::onOrientationReadingChanged()
{
    m_orientation_reader = m_orientation_sensor->reading();
    mysocket->Orient  = m_orientation_reader->orientation();// x();  // when rotated...
}

void MainWindow::onRotationReadingChanged()
{
    m_rotation_reader = m_rotation_sensor->reading();
    mysocket->AngleX  = -m_rotation_reader->x();        // Roll...
    mysocket->AngleY  = -m_rotation_reader->y()-90;     // Pitch...
    mysocket->AngleZ  = m_rotation_reader->z(); // Yaw...
 //   qDebug() << mysocket->AngleX << mysocket->AngleY << mysocket->AngleZ;
}

// Source - https://stackoverflow.com/a/10990973
// Posted by jxh, modified by community. See post 'Timeline' for change history
// Retrieved 2026-03-16, License - CC BY-SA 3.0

template <unsigned N>
double approxRollingAverage (double avg, double input) {
    avg -= avg/N;
    avg += input/N;
    return avg;
}


void MainWindow::AccelerometerRead()
{
//#define GFILTER 100
    static double avg=9.82500f;
    double tmp=0;

    if(m_calibrate > 0)
    {
        tmp = sqrt(mysocket->AccX*mysocket->AccX +
                   mysocket->AccY*mysocket->AccY +
                   mysocket->AccZ*mysocket->AccZ);

        if(tmp != 0)
            avg = approxRollingAverage<20>(avg, tmp);
        else
            m_calibrate = 1;

        qDebug() << "tmP / AVG: " << tmp << " -> " << avg;;

        if(m_calibrate == 1)
        {
            if(mysocket->IMUconnected == false){
#ifdef Q_OS_MAC
                m_first = 6;
#else
                m_first = 200;
#endif
            }
            else{
                m_first = 6;
            }
            // Store install orientation...
            m_install = Vector3d(0,-mysocket->AngleY*DEG_TO_RAD,0);

            mysocket->G         = avg;
            ekf.ekf->Gval       = avg;
            ekf.ekf_quart->Gval = avg;
            ekf.Gval            = avg;

            Matrix3x6 config;
            if(get_default_config(config) == 0)
            {
                config[0][2] = (double)avg;
                set_default_config(config);
                qDebug() << "Stored new config!!! " << avg;
            }
        }
        m_calibrate--;
    }
}

void MainWindow::onPressureReadingChanged()
{
    if(m_pressure_sensor != nullptr){
        m_pressure_reader = m_pressure_sensor->reading();
        this->mysocket->m_pressure_raw = m_pressure_reader->pressure()/100.0;
    }else{
        this->mysocket->m_pressure_raw = m_pressure_reader->pressure()/100.0;
    }
    afterPressureReadingChanged();
}

void MainWindow::afterPressureReadingChanged()
{
    static int first = 30;

    // If we got an altitude...
    if( this->mysocket->m_altitude > 0.1)
    {
        // Sett takeoff altitude automatically...
        if(first){
            first--;
            if(first == 1){
                on_reset_altitude_2_clicked();
            }
        }
    }

    // --- Runtime computation using the UI offset ---
    const double qnh_hpa_ui = ui->doubleSpinBox->text().toDouble();  // sea-level pressure
//    const double qnh_hpa_ui = m_pressure_raw + (ui->doubleSpinBox->text().toDouble() - 1013.25);  // sea-level pressure

    //  Meters: 44330.0f * (1.0f - pow(pressure_hPa / qnh, 0.1903f));
    // Feet ...
    this->mysocket->m_preasure_alt = 145366.45 * (1.0 - std::pow(this->mysocket->m_pressure_raw / qnh_hpa_ui, 0.190284));
    ui->baro_alt->setText(QString::number(this->mysocket->m_preasure_alt));
}

double MainWindow::setQNH()
{
    if( this->mysocket->m_altitude > 0.1)
    {
        // Known altitude in feet (e.g. GPS)
        double feet_gps = this->mysocket->m_altitude;

        // Calculate the sea-level pressure in hPa
        double qnh_hpa = this->mysocket->m_pressure_raw /
                         std::pow(1.0 - (feet_gps / 145366.45), 1.0 / 0.190284);

        // Pressure offset in millibars (hPa) to add to m_pressure_raw
        double pressure_offset = qnh_hpa - this->mysocket->m_pressure_raw;

        //  Meters: 44330.0f * (1.0f - pow(pressure_hPa / qnh, 0.1903f));
        // Feet ...
        // Check: this should now yield GPS altitude
        mysocket->m_preasure_QNH  = 145366.45 * (1.0 - std::pow(this->mysocket->m_pressure_raw / qnh_hpa, 0.190284));

        qDebug() << "GPS ALT:" <<feet_gps << " Press.Error:" << pressure_offset << " Raw.Press:"
                 << this->mysocket->m_pressure_raw<< " qnh-> " << qnh_hpa
                 << " Calc.GPS.Alt:" << mysocket->m_preasure_QNH; // << " Test: " << test;

        ui->doubleSpinBox->setText(QString("%1").arg(qnh_hpa));

       _widgetALT->setPressure((float)qnh_hpa/100.0);
        _widgetALT->redraw();

        return qnh_hpa;
    }
    else return 1013.25;
}

// Returns bearing in degrees (0° = North, 90° = East, etc.)
double MainWindow::getBearing(double lat1, double lon1, double lat2, double lon2)
{
    // Convert degrees to radians
    double φ1 = qDegreesToRadians(lat1);
    double φ2 = qDegreesToRadians(lat2);
    double Δλ = qDegreesToRadians(lon2 - lon1);

    double y = qSin(Δλ) * qCos(φ2);
    double x = qCos(φ1) * qSin(φ2) - qSin(φ1) * qCos(φ2) * qCos(Δλ);
    double θ = qAtan2(y, x);

    double bearing = qRadiansToDegrees(θ);
    return fmod((bearing + 360.0), 360.0); // Normalize to 0-360°
}

// GPS calculations...
void MainWindow::positionUpdated(QGeoPositionInfo geoPositionInfo)
{
    // If we have a GPS lock...
    if (geoPositionInfo.isValid() && mysocket->m_altitude < 1)
    {
        double vel_D=0;

        //locationDataSource->stopUpdates();
        QGeoCoordinate geoCoordinate = geoPositionInfo.coordinate();
        this->mysocket->m_latitude  = geoCoordinate.latitude();
        this->mysocket->m_longitude = geoCoordinate.longitude();

        // ... TERJE
        static GeoidHelper geo;
        auto result = geo.compensatedHeight(this->mysocket->m_latitude , this->mysocket->m_longitude, geoCoordinate.altitude(), 0.5);
        /*
        if (result) {
            qDebug() << "Geoid separation (N):" << result->N
                     << "Compensated height:" << result->h_compensated;
        }
        */
        if(result.has_value())
            this->mysocket->m_altitude  = result->h_compensated;

        // BE AWARE THIS IS WGS-84 prox. compensated N AND NOT AMSL (see level) ...
        //      this->m_altitude  = (geoCoordinate.altitude()-40.0)*3.2808399;
        //    qDebug() << "GPS Alt:  " << this->m_altitude ;

        this->mysocket->m_gpsspeed  = geoPositionInfo.attribute(QGeoPositionInfo::GroundSpeed);
        mysocket->m_gpsbearing      = geoPositionInfo.attribute(QGeoPositionInfo::Direction);
        vel_D                       = -geoPositionInfo.attribute(QGeoPositionInfo::VerticalSpeed);

        calcPosition(vel_D);
    }
}

void MainWindow::calcPosition(double vel_D)
{
    bool   has_pos = true;
    static bool first = true;

    static steady_clock::time_point clock_begin = steady_clock::now();
    steady_clock::time_point clock_end = steady_clock::now();
    steady_clock::duration time_span = clock_end - clock_begin;
    double dt = double(time_span.count()) / 1000000000L;

    clock_begin=clock_end;

    // Fi we are simulation GPS (and we do not use MQTT)
    if(simGPS == true && !mysocket->m_has_MQTT)
    {
 //       static bool first = true;
        static QList<TrackPoint> points;

        // IF this is the boot time, we read inn the stimuli data from the simulation file...
        if(first == true)
        {
            first = false;
            GpxParser parser;
            if (parser.parseFile(":/sin.gpx"))
            {
//                if (parser.parseFile(":/example.gpx")) {
                points = parser.getTrackPoints();
            }
        }

        // IF we found some waypoints...
        if(!points.isEmpty())
        {
            // Set up som integrators...
            static double l_speed=0;
            static double l_delta_speed = 0.08;

            // Get next position...
            has_pos = true;
            TrackPoint pt = points.takeFirst();

            // From last position to current position what was the bearing...
            this->mysocket->m_gpsbearing= getBearing(this->mysocket->m_latitude,this->mysocket->m_longitude,pt.latitude,pt.longitude);
            // Upside down for testing south direction...
  //          m_gpsbearing      = getBearing(pt.latitude,pt.longitude,this->m_latitude,this->m_longitude);

            this->mysocket->m_latitude  = pt.latitude;
            this->mysocket->m_longitude = pt.longitude;

            // ... TERJE
            static GeoidHelper geo;
            auto result = geo.compensatedHeight(this->mysocket->m_latitude , this->mysocket->m_longitude, pt.elevation, 0.5);
            /*
            if (result) {
                qDebug() << "Geoid separation (N):" << result->N
                         << "Compensated height:" << result->h_compensated
                         << "Bearing:" << this->m_gpsbearing;
            }
            */
            if(result.has_value())
                this->mysocket->m_altitude  = result->h_compensated*3.2808399;

          //  this->m_altitude  = pt.elevation*3.2808399; // Make feet from meters...
            this->mysocket->m_gpsspeed  = 25+(sin(l_speed)*20); //pt.speed;  // For now we use a created speed...
            dt                = pt.dt;
            vel_D             = 0;

            l_speed+=l_delta_speed;
/*
            qDebug() << "Lat:" << pt.latitude
                     << "Lon:" << pt.longitude
                     << "Ele:" << pt.elevation
                     << "Speed:" << pt.speed
                     << "Bearing:" << m_gpsbearing
                     << "Time:" << pt.time.toString(Qt::ISODate);
*/
        }
        // If end of file restart...
        else{
            first = true;
             has_pos = false;
        }
    }

    if(has_pos == true)
    {
        m_geopos = true;
        mysocket->m_speed    = this->mysocket->m_gpsspeed*3.6;

        if(this->mysocket->m_gpsspeed > 2.5 && !isnan(mysocket->m_gpsbearing))
        {
            static double old_bearing = 0;

            m_bearing    = mysocket->m_gpsbearing;

            double delta_bearing = m_bearing - old_bearing;
            if (delta_bearing > 180) delta_bearing -= 360;
            if (delta_bearing < -180) delta_bearing += 360;

            double turn_rate_derivated = delta_bearing / dt;

            // Filter
            static double filtered_turn_rate = 0;
            double alpha = 0.1;
            filtered_turn_rate = alpha * turn_rate_derivated + (1 - alpha) * filtered_turn_rate;

            double a_c_exp = this->mysocket->m_gpsspeed * filtered_turn_rate * DEG_TO_RAD;
            m_roll_angle = -atan2(a_c_exp, ekf.Gval);

            m_total_accel = sqrt(ekf.Gval); //sqrt((ekf.Gval*ekf.Gval)); // + (a_c_exp*a_c_exp)); //    # magnitude of net acceleration
            old_bearing = m_bearing;

            // correct for turn and acceleration...
            roll_blended = m_roll_angle;
            roll_blended_ok = true;

        }
        else{
            m_bearing     = 999;
            m_total_accel = 0;
            m_roll_angle  = 0;
            m_accel_body  = {0,0,0};
            roll_blended_ok = false;
        }
        // Get velocity vector...
        if(!(isnan(this->mysocket->m_gpsspeed) || isnan(this->mysocket->m_gpsbearing) || isnan(vel_D)))
        {
            this->mysocket->m_vel_N  = cos(mysocket->m_gpsbearing*DEG_TO_RAD)*this->mysocket->m_gpsspeed;
            this->mysocket->m_vel_E  = sin(mysocket->m_gpsbearing*DEG_TO_RAD)*this->mysocket->m_gpsspeed;
            this->mysocket->m_vel_D  = vel_D;
            this->mysocket->m_vel_active = true;
        }
        else{
            this->mysocket->m_vel_E=this->mysocket->m_vel_N=this->mysocket->m_vel_D=0.0;
            this->mysocket->m_vel_active = false;
        }
    }
    else{
        qDebug() << "No GPS data at: " << dt;
    }
}

void MainWindow::EKF()
{
    if((m_gyro_reader != nullptr && m_accel_reader != nullptr) || (ekf.m_use_gpt == 0) || m_use_imu )
    {
        static steady_clock::time_point clock_begin = steady_clock::now();
        steady_clock::time_point clock_end = steady_clock::now();
        steady_clock::duration time_span = clock_end - clock_begin;
        m_dt = double(time_span.count()) / 1000000000L;
        clock_begin=clock_end;

        // How to caulcate Euler Angles [Roll Φ(Phi) Gyro Z, Pitch θ(Theta) gyro Y, Yaw Ψ(Psi) Gyro X]
        if(ekf.m_use_gpt > 0 && mysocket->m_has_MQTT == false)
        {
          //  if(m_mag_reader == nullptr){
          //      mysocket->HX=mysocket->HY=mysocket->HZ=0;
          //  }
            Vector3d gyro = {mysocket->AsZ,mysocket->AsY,-mysocket->AsX};
            Vector3d accel= {-mysocket->AccY,-mysocket->AccZ,mysocket->AccX};
            Vector3d mag  = {mysocket->HX,mysocket->HY,mysocket->HZ};

            // Compensate for mounting angle (only tilt).
            // Counter-rotate in the oppositt direction of the mounting found at start...
            gyro = ekf.ekf_quart->mountingRot(1,gyro, m_install);
            accel = ekf.ekf_quart->mountingRot(0,accel, -m_install);
            mag = ekf.ekf_quart->mountingRot(0,mag, -m_install);
        //    qDebug() << m_install[0]*RAD_TO_DEG << m_install[1]*RAD_TO_DEG << m_install[2]*RAD_TO_DEG;

            // remove forces calculated by GPS...
            if(m_use_gps_in_attitude)
            {
                accel = accel - m_accel_body;
            }

            float dummy;            

            // Estimate using accelerometers...
            std::tie(a_pitch,a_roll,dummy) = ekf.getPitchRoll(accel[1],accel[0],accel[2],ekf.Gval);
            m_yaw = ekf.getHeading(mag[1],mag[0],mag[2], ekf.getRoll_rad(),ekf.getPitch_rad());

            if(isnan(m_yaw)) m_yaw = 0;
            if(isnan(a_pitch)) a_pitch = 0;
            if(isnan(a_roll)) a_roll = 0;
            if(isnan(a_yaw)) a_yaw = 0;

            // Calculates X and Y relative distances in meters.
            double ypos = 0;
            double xpos = 0;
            if(!(isnan(this->mysocket->m_latitude) || isnan(this->mysocket->m_longitude)))
            {
                double deltaLatitude = this->mysocket->m_latitude - this->takeoff_latitude;
                double deltaLongitude = this->mysocket->m_longitude - this->takeoff_longitude;
                double latitudeCircumference = 40075160 * cos(this->takeoff_latitude*DEG_TO_RAD);
                ypos = deltaLongitude * latitudeCircumference / 360;
                xpos = deltaLatitude * 40008000 / 360;
            }

            // get speed vector...
            double vel_N = 0.0;
            double vel_E = 0.0;
            double vel_D = 0.0;
            if (this->mysocket->m_vel_active == true)
            {
                vel_N = this->mysocket->m_vel_N;
                vel_E = this->mysocket->m_vel_E;
                vel_D = this->mysocket->m_vel_D;
            }

            // update kalmanfilter
            ekf.ekf_update( m_dt,
                           vel_N,
                           vel_E,
                           vel_D,
                           xpos, //this->m_latitude, //xpos, // * 1e-7,
                           ypos, //this->m_longitude, //ypos, // * 1e-7,
                           -this->mysocket->m_altitude, // * 1e-3,
                           -this->mysocket->m_preasure_alt*0.3048, // Meter NED...
                           -gyro[1]* DEG_TO_RAD,
                           gyro[0]* DEG_TO_RAD, //gyro[0] * DEG_TO_RAD,
                           -gyro[2]* DEG_TO_RAD, // gyro[2] * DEG_TO_RAD,
                           a_pitch,
                           a_roll,
                           m_yaw, //a_yaw, // m_yaw,
                           // Adding magnetometer will help on long turns, however they are unpredictable.
                           0, //m_pitch,
                           0, //m_roll,
                           0 //m_yaw
                        );

#define SCALE 1.00
            Vector3d attitude = {ekf.getRoll_rad(),ekf.getPitch_rad(),ekf.getHeading_rad()};
            m_attitude = attitude * RAD_TO_DEG;
            m_heading = ekf.getHeading_rad()* RAD_TO_DEG;

            if(roll_blended_ok)
            {
                // correct for turn ...
                double r = roll_blended = 0.95 * m_attitude[0] + 0.05 * roll_blended; // complementary filter
                m_attitude[0] = r;
           }
        }
        else{
            Vector3d attitude = {mysocket->AngleX,
                                 mysocket->AngleY,
                                 mysocket->AngleZ};
            m_attitude = attitude;
            m_heading = mysocket->AngleZ;

        }
    }
}

void MainWindow::onReadingChanged()
{
#define filterlength 30
    static double filter[filterlength]={0};
#define rotfilterlength 20
//    static double rotfilter[rotfilterlength]={0};
#define headfilterlength 3
//    static double headfilter[headfilterlength]={0};
    double x_head = 0.0;
    double slipp = 0.0;
//    static bool running = false;

    // Sensor rotation...
    if(m_calibrate != 0)
    {
        AccelerometerRead();
    }

    // Run code...
    {
        Vector3d gyro = {mysocket->AsZ,mysocket->AsY,mysocket->AsX};
       // Vector3d accel= {-mysocket->AccY,-mysocket->AccZ,mysocket->AccX};
        Vector3d attitude  = m_attitude;

        Vector3d accel= {-mysocket->AccY,-mysocket->AccZ,mysocket->AccX};
        accel = ekf.ekf_quart->mountingRot(0,accel, -m_install);

     //   qDebug() << accel[0] << accel[1] << accel[2] << mysocket->G;

        if(m_first > 0)
        {
            m_first--;

            if(m_first == 5 && splash != nullptr ){
#if defined(Q_OS_IOS) || defined(Q_OS_MACOS)
                splash->hide(); // Hides splash screen
                splash->finish(this); // Hides splash screen
                delete(splash);
                splash = nullptr;
#endif
                m_msgBox->hide();
            }
            /*
            if(m_first == 73){
                m_msgBox->hide();
                QCoreApplication::processEvents();
                m_msgBox->show();
                QCoreApplication::processEvents();
            }
*/
            if(m_first == 1){
                if(!this->m_msgBox->isHidden())
                {
                    this->m_msgBox->hide();
                }

                if(!(this->m_msgBoxCalibrating == nullptr))
                {
                    this->m_msgBoxCalibrating->hide();
                    delete(this->m_msgBoxCalibrating);
                    this->m_msgBoxCalibrating = nullptr;
                }

                m_offset = attitude[1];
                m_acc_Y_calib = accel[0];

                //m_pitch_cal+= a_pitch - m_pitch;
                //m_roll_cal+= a_roll - m_roll;
            }
        }

        if(m_bearing != 999){
            x_head = m_bearing;
        }
        else{
            x_head = m_heading - heading_offset;
        }
//        slipp = accel[1]*9.5;
        slipp = (accel[0]-m_acc_Y_calib)*9.5;

        if(mysocket->Transponderstat == false &&  ui->radioButton->isCheckable())
        {
            ui->radioButton->setChecked(false);
            ui->radioButton->setCheckable(true);
            ui->radioButton_3->setChecked(false);
        }
        else if(mysocket->Transponderstat == true &&  !ui->radioButton->isCheckable())
        {
            ui->radioButton->setCheckable(true);
        }
/*
        // Make compass...
        //-----------------
        static double filtered_h = 0;  // Persistent filtered value
        double alpha_h = 0.01;            // Smoothing factor (0 < alpha < 1)

        double h = x_head;            // Current raw gyro reading
        filtered_h = alpha_h * h + (1 - alpha_h) * filtered_h;

        double head_dir = filtered_h; // This is your smoothed gyro reading
        //-----------------
*/
        /*
        double head_dir = 0;
        for(int x=0; x < headfilterlength-1;x++){
            headfilter[x] = headfilter[x+1];
            head_dir+=headfilter[x];
        }
        headfilter[headfilterlength-1] = x_head;
        head_dir+=x_head;
        head_dir/=headfilterlength;
        */
#ifdef USE_ANGLE
        if(mysocket->Anglestat == true){
            m_head = this->mysocket->AngleSensor;
        }
        else{
            m_head = 0;
        }
#else
        m_head = x_head; //head_dir;
#endif
        //-----------------

        // Store data in local registers...
        double roll_att = 0;
        double pitch_att = 0;

        roll_att  = -attitude[0];
        pitch_att = 1 * (attitude[1] - m_offset);

        //-----------------
        static double filtered_r = 0;  // Persistent filtered value
        double alpha = 0.02;            // Smoothing factor (0 < alpha < 1)
        double r = gyro[2]; //1]; // should be 2           // Current raw gyro reading
        if(r <  0.5 && r > 0){alpha = 0.25;}
        if(r > -0.5 && r < 0){alpha = 0.25;}
        filtered_r = alpha * r + (1 - alpha) * filtered_r;
        double rot_speed = -filtered_r; // This is your smoothed gyro reading
        //-----------------

        /*
        double r = gyro[2];// *2; //((sin(attitude[2]*DEG_TO_RAD)*gyro[0])+(cos(attitude[2]*DEG_TO_RAD)*gyro[2]));
        if(r >  6.0)r =  6.0;
        if(r < -6.0)r = -6.0;

        double rot_speed = 0;
        for(int x=0; x < rotfilterlength-1;x++){
            rotfilter[x] = rotfilter[x+1];
            rot_speed+=rotfilter[x];
        }
        rotfilter[rotfilterlength-1] = r;
        rot_speed+=r;
        rot_speed/=-rotfilterlength;
        */
        //-----------------

        // Make the turn bank ...
        double slippIndicator = 0;
        if(slipp >  16.0)slipp =  16.0;
        if(slipp < -16.0)slipp = -16.0;
        for(int x=0; x < filterlength-1;x++)
        {
            filter[x] = filter[x+1];
            slippIndicator+=filter[x];
        }
        filter[filterlength-1] = slipp;
        slippIndicator+=filter[filterlength-1];
        slippIndicator/=filterlength;

        // Radar...
        static int memory = 0;
        static std::deque<float> dq = {0};
        static std::deque<float> sq = {0};
        //static int lastDetection = 0;
        int scale = Radar_Height / 100;

        //if(lastDetection != mysocket->rPos)
        {
        //    lastDetection = mysocket->rPos;
            dq.push_back(20.0+(mysocket->rDist*scale));
            sq.push_back(20.0+(mysocket->rSpeed*scale));
            if(++memory >= Radar_depth)
            {
                dq.pop_front();
                sq.pop_front();
                memory = Radar_depth;
            }
        }
        // .......


        if( currentIndex == P_FLIGT_INSTRUMENT)
        {
            static int xtimer=0;

            if(++xtimer%3 == 0)
            {
                if( ui->radioButton_3->isChecked()){
                    _widgetALT->setAltitude(this->mysocket->m_preasure_alt);
                }else if( ui->radioButton_2->isChecked())
                {
                    if(this->mysocket->m_altitude > -100.0){
                        _widgetALT->setAltitude(this->mysocket->m_altitude);
                    }
                    else{
                        _widgetALT->setAltitude(0);
                    }
                }else{
                    _widgetALT->setAltitude(m_tansALT);
                }

                _widgetALT->redraw();

                if(this->mysocket->Airspeed_data.airspeed > -1){
                    // If we got airspeed then change display to airspeed...
                    if(!_widgetASI->getver()){
                        _widgetASI->reinit(1);
                    }
                    _widgetASI->setAirspeed(mysocket->Airspeed_data.airspeed);
                }
                else if(mysocket->m_speed >= 0 && mysocket->m_speed < 300){
                    // If we do NOT got airspeed then change display to groundspeed...
                    if(_widgetASI->getver()){
                        _widgetASI->reinit(0);
                    }
                    _widgetASI->setAirspeed(mysocket->m_speed);
                }
                else
                    _widgetASI->setAirspeed(0);

                _widgetASI->redraw();

                _widgetHI->setHeading(abs(m_head-359.9));
                _widgetHI->redraw();
            }

            _widgetAI->setRoll ( roll_att );
            //_widgetAI->setRoll ( m_roll_angle*RAD_TO_DEG); // JUST FOR NOT; GPS ROLL...

            _widgetAI->setPitch( pitch_att );
            _widgetAI->redraw();

            if(xtimer%2 == 0)
            {
                _widgetTC->setTurnRate(rot_speed);
                _widgetTC->setSlipSkid(slippIndicator);
                _widgetTC->redraw();

                _widgetVSI->setClimbRate(m_var_speed);
                _widgetVSI->redraw();
            }
        }

        if( currentIndex == P_AUTOPILOT)
        {
            ui->label_Bearing->setText("10.0");
            ui->label_Bearing->raise();
            ui->label_Pitch->setText("11.0");
            ui->label_Pitch->raise();
            ui->label_Power->setText("12.0");
            ui->label_Power->raise();
            ui->label_Roll->setText("13.0");
            ui->label_Roll->raise();
            ui->label_Pitch->setText("14.0");
            ui->label_Pitch->raise();
//            ui->label_UTC->setText("15.0");
            ui->label_UTC->raise();
            ui->label_TMP->setText("16.0");
            ui->label_TMP->raise();
            ui->label_Speed->setText("17.0");
            ui->label_Speed->raise();
        }

        if( currentIndex == P_GLASS_COCPIT)
        {
            if( ui->radioButton_3->isChecked())
            {
                _widgetEADI->setAltitude( this->mysocket->m_preasure_alt );
            }else if( ui->radioButton_2->isChecked())
            {
                _widgetEADI->setAltitude(this->mysocket->m_altitude);
            }else{
                _widgetEADI->setAltitude( m_tansALT);
            }
            _widgetEADI->setClimbRate(m_var_speed);
            _widgetEADI->setPressure( this->mysocket->m_preasure, qfi_EADI::PressureMode::MB );

            _widgetEADI->setHeading( m_head );
            _widgetEADI->setSlipSkid( slippIndicator/57 );
            _widgetEADI->setTurnRate( rot_speed );
            _widgetEADI->setRoll( roll_att );
            _widgetEADI->setPitch(pitch_att );

            _widgetEADI->setFltMode( qfi_EADI::FltMode::CMD);
            _widgetEADI->setSpdMode( qfi_EADI::SpdMode::FMC_SPD);
            _widgetEADI->setLNAV( qfi_EADI::LNAV::APR_ARM);
            _widgetEADI->setVNAV( qfi_EADI::VNAV::ALT);
            _widgetEADI->setFPM( 0.0, 0.0 );
            _widgetEADI->setDots( 0.0,0.0,false,false);
            _widgetEADI->setFD( 0.0,0.0,true);
            _widgetEADI->setStall( false );
            _widgetEADI->setMachNo( 0.0 );
            _widgetEADI->setAirspeedSel( 80.0 );
            _widgetEADI->setAltitudeSel( 400 );
            _widgetEADI->setHeadingSel( 80.0 );
            _widgetEADI->setVfe( 120.0 );
            _widgetEADI->setVne( 160.0 );
            _widgetEADI->redraw();

            _widgetEHSI->setHeading( 0 );
            _widgetEHSI->setCourse( m_head );
            _widgetEHSI->setBearing( m_head, true );
            _widgetEHSI->setDeviation( 0.0, qfi_EHSI::CDI::FROM);
            _widgetEHSI->setDistance( 200, true );
            _widgetEHSI->setHeadingSel( m_head );
            _widgetEHSI->redraw();
        }

        if( currentIndex == P_RADAR)
        {
            if(mysocket->Radarstat == true)
            {

                auto *view = ui->graphicsView_2;                 // <-- use one view consistently
                QGraphicsScene *m_graphScen = view->scene();
                if (!m_graphScen) {
                    m_graphScen = new QGraphicsScene(view);
                    view->setScene(m_graphScen);
                }

                QPixmap pix2(":/images/radar_3.png");                      // from resources
                if (pix2.isNull()) {
                    qWarning() << "Failed to load :/radar_3.png; check .qrc path";
                    return;
                }

                m_graphScen->clear();
                QGraphicsPixmapItem *item = m_graphScen->addPixmap(pix2);
                item->setTransformationMode(Qt::SmoothTransformation);
                item->setOpacity(0.5);  //
                m_graphScen->setSceneRect(item->boundingRect());
                view->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
                view->fitInView(item, Qt::IgnoreAspectRatio);
                QTimer::singleShot(0, view, [view, item]{
                    view->fitInView(item, Qt::IgnoreAspectRatio);
                });

                // Define scene rect from the image
                QRectF r = item->boundingRect();      // scene units
                const qreal left = 0; //r.left();
                const qreal right = r.right();
                const qreal top = 0; //r.top();
                const qreal bottom = r.bottom();
                const qreal height = r.height()/12;
                Radar_depth = right/8;
                Radar_Height = r.height();

                auto dashed = [](QColor c) {
                    c.setAlpha(80);
                    return QPen(QBrush(c), 1, Qt::DotLine);
                };
                m_graphScen->addLine(left + 5,  top + (height*1), right, top + (height*1), dashed(Qt::green));
                m_graphScen->addLine(left + 5,  top + (height*2), right, top + (height*2), dashed(Qt::green));
                m_graphScen->addLine(left + 5,  top + (height*3), right, top + (height*3), dashed(Qt::green));
                m_graphScen->addLine(left + 5,  top + (height*4), right, top + (height*4), dashed(Qt::green));
                m_graphScen->addLine(left + 5,  top + (height*5), right, top + (height*5), dashed(Qt::green));
                m_graphScen->addLine(left + 5,  top + (height*6), right, top + (height*6), dashed(Qt::cyan));
                m_graphScen->addLine(left + 5,  top + (height*7), right, top + (height*7), dashed(Qt::cyan));
                m_graphScen->addLine(left + 5,  top + (height*8), right, top + (height*8), dashed(Qt::yellow));
                m_graphScen->addLine(left + 5,  top + (height*9), right, top + (height*9), dashed(Qt::yellow));
                m_graphScen->addLine(left + 5,  top + (height*10), right, top + (height*10), dashed(Qt::yellow));
                m_graphScen->addLine(left + 5,  top + (height*11), right, top + (height*11), dashed(Qt::red));
                m_graphScen->addLine(left + 5,  top + (height*12), right, top + (height*12), dashed(Qt::red));

                // Example points series (memory, dq, sq in scene units)
                for (int i = 0; i < memory-1; ++i) {
                    qreal height_pos = dq[i]+20;
                //    qreal speed_pos  = sq[i];

                    qreal x = left + i * 8;          // 6 scene units per step
                    m_graphScen->addLine(x, bottom - height_pos, x + 7, bottom - height_pos,
                                   QPen(QBrush(Qt::yellow), 4));
               //     m_graphScen->addLine(x, bottom - speed_pos,  x + 7, bottom - speed_pos,
                 //                  QPen(QBrush(Qt::white), 4));
                }

                ui->label_26->setText(QString::number(mysocket->rDist));
                ui->label_29->setText(QString::number(mysocket->rSpeed));
            }
        }

        if( currentIndex == P_IMU)
        {
            QGraphicsScene * m_graphScen = new QGraphicsScene;
            QSize x = ui->graphicsView->size();

            m_graphScen->setSceneRect(0,0,x.width(),x.height());
            float x1 = cos(-roll_att/(180.0/3.1415));
            float y1 = sin(-roll_att/(180.0/3.1415));
         //   float z1 = sin(pitch_att/(180.0/3.1415));

            int offset = pitch_att;
            if (offset > 40) offset = 40;
            if (offset < -40) offset = -40;
            offset = offset * (x.height()/180.0);

            // -----------------------------------------
            // Artificial Horizon line...
            m_graphScen->addLine((int)((x.width() /2)-(x1*200)),
                                 (int)((x.height()/2)-(y1*200))+offset,
                                 (int)((x.width() /2)+(x1*200)),
                                 (int)((x.height()/2)+(y1*200))+offset,
                                 QPen(QBrush(Qt::yellow),6));

            if( roll_att > 60 || roll_att < -60 || pitch_att > 30 || pitch_att < -30)
            {
                ui->graphicsView->setBackgroundBrush(QBrush(Qt::red));
            }
            else
            {
                ui->graphicsView->setBackgroundBrush(QBrush(Qt::black));
            }

            // -----------------------------------------
            // Horisontal center line...
            m_graphScen->addLine((int)5,
                                 (int)((x.height()/2)-0),
                                 (int)x.width()-5,
                                 (int)((x.height()/2)+0),
                                 QPen(QBrush(Qt::green),2,Qt::PenStyle(Qt::DashLine)));

            // Vertical center line...
            m_graphScen->addLine((int)(x.width() /2),
                                 (int) 5,
                                 (int)(x.width() /2),
                                 (int)x.height()-5,
                                 QPen(QBrush(Qt::green),2,Qt::PenStyle(Qt::DashLine)));

            // Horisontal bottom line...
            m_graphScen->addLine((int)175,
                                 (int)((x.height()/1)-100),
                                 (int)x.width()-175,
                                 (int)((x.height()/1)-100),
                                 QPen(QBrush(Qt::darkMagenta),8));

            m_graphScen->addLine((int)175,
                                 (int)((x.height()/1)-100),
                                 (int)x.width()-175,
                                 (int)((x.height()/1)-100),
                                 QPen(QBrush(Qt::white),1));

            // Horisontal bottom line...
            m_graphScen->addEllipse((slippIndicator*4)+((x.width()/2)-11),x.height()-110,20,20,QPen(Qt::white),QBrush(Qt::cyan));

            // -----------------------------------------
            // Center main angle...
            m_graphScen->addLine((int)((x.width() /2)-(x1*60)),
                                 (int)((x.height()/2)-(y1*60)),
                                 (int)((x.width() /2)+(x1*60)),
                                 (int)((x.height()/2)+(y1*60)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Center main angle...
            float p_loc_A = (x.width() /2) + ((int)m_head % 60);
            float p_loc_B = (x.height() /2);

            m_graphScen->addLine((int)(p_loc_A),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A-60),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A-60),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A+60),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A+60),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A-120),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A-120),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A+120),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A+120),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A-180),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A-180),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            m_graphScen->addLine((int)(p_loc_A+180),
                                 (int)(p_loc_B-10),
                                 (int)(p_loc_A+180),
                                 (int)(p_loc_B+10),
                                 QPen(QBrush(Qt::white),3,Qt::PenStyle(Qt::SolidLine)));

            // First upper bar...
            p_loc_A = (x.width() /2)  + (y1*15.0);
            p_loc_B = (x.height() /2) - (x1*15.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*50)),
                                 (int)(p_loc_B-(y1*50)),
                                 (int)(p_loc_A+(x1*50)),
                                 (int)(p_loc_B+(y1*50)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Second upper bar...
            p_loc_A = (x.width() /2)  + (y1*30.0);
            p_loc_B = (x.height() /2) - (x1*30.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*40)),
                                 (int)(p_loc_B-(y1*40)),
                                 (int)(p_loc_A+(x1*40)),
                                 (int)(p_loc_B+(y1*40)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Third upper bar...
            p_loc_A = (x.width() /2)  + (y1*45.0);
            p_loc_B = (x.height() /2) - (x1*45.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*30)),
                                 (int)(p_loc_B-(y1*30)),
                                 (int)(p_loc_A+(x1*30)),
                                 (int)(p_loc_B+(y1*30)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Forth upper bar...
            p_loc_A = (x.width() /2)  + (y1*60.0);
            p_loc_B = (x.height() /2) - (x1*60.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*20)),
                                 (int)(p_loc_B-(y1*20)),
                                 (int)(p_loc_A+(x1*20)),
                                 (int)(p_loc_B+(y1*20)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // First lower bar...
            p_loc_A = (x.width() /2)  - (y1*15.0);
            p_loc_B = (x.height() /2) + (x1*15.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*50)),
                                 (int)(p_loc_B-(y1*50)),
                                 (int)(p_loc_A+(x1*50)),
                                 (int)(p_loc_B+(y1*50)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Second lower bar...
            p_loc_A = (x.width() /2)  - (y1*30.0);
            p_loc_B = (x.height() /2) + (x1*30.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*40)),
                                 (int)(p_loc_B-(y1*40)),
                                 (int)(p_loc_A+(x1*40)),
                                 (int)(p_loc_B+(y1*40)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // Third lower bar...
            p_loc_A = (x.width() /2)  - (y1*45.0);
            p_loc_B = (x.height() /2) + (x1*45.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*30)),
                                 (int)(p_loc_B-(y1*30)),
                                 (int)(p_loc_A+(x1*30)),
                                 (int)(p_loc_B+(y1*30)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));
            // Forth lower bar...
            p_loc_A = (x.width() /2)  - (y1*60.0);
            p_loc_B = (x.height() /2) + (x1*60.0);

            m_graphScen->addLine((int)(p_loc_A-(x1*20)),
                                 (int)(p_loc_B-(y1*20)),
                                 (int)(p_loc_A+(x1*20)),
                                 (int)(p_loc_B+(y1*20)),
                                 QPen(QBrush(Qt::white),2,Qt::PenStyle(Qt::DashLine)));

            // -----------------------------------------

            m_graphScen->addLine((int)((x.width() /2)-100),
                                 (int)((x.height()/2)-19+offset),
                                 (int)((x.width() /2)-80),
                                 (int)((x.height()/2)-15+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)+100),
                                 (int)((x.height()/2)-19+offset),
                                 (int)((x.width() /2)+80),
                                 (int)((x.height()/2)-15+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)-100),
                                 (int)((x.height()/2)+19+offset),
                                 (int)((x.width() /2)-80),
                                 (int)((x.height()/2)+15+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)+100),
                                 (int)((x.height()/2)+19+offset),
                                 (int)((x.width() /2)+80),
                                 (int)((x.height()/2)+15+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));


            m_graphScen->addLine((int)((x.width() /2)+80),
                                 (int)((x.height()/2)+40+offset),
                                 (int)((x.width() /2)+60),
                                 (int)((x.height()/2)+30+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)-80),
                                 (int)((x.height()/2)+40+offset),
                                 (int)((x.width() /2)-60),
                                 (int)((x.height()/2)+30+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)+80),
                                 (int)((x.height()/2)-40+offset),
                                 (int)((x.width() /2)+60),
                                 (int)((x.height()/2)-30+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)-80),
                                 (int)((x.height()/2)-40+offset),
                                 (int)((x.width() /2)-60),
                                 (int)((x.height()/2)-30+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));


            m_graphScen->addLine((int)((x.width() /2)-60),
                                 (int)((x.height()/2)-60+offset),
                                 (int)((x.width() /2)-45),
                                 (int)((x.height()/2)-45+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)+60),
                                 (int)((x.height()/2)-60+offset),
                                 (int)((x.width() /2)+45),
                                 (int)((x.height()/2)-45+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)+60),
                                 (int)((x.height()/2)+60+offset),
                                 (int)((x.width() /2)+45),
                                 (int)((x.height()/2)+45+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            m_graphScen->addLine((int)((x.width() /2)-60),
                                 (int)((x.height()/2)+60+offset),
                                 (int)((x.width() /2)-45),
                                 (int)((x.height()/2)+45+offset), QPen(QBrush(Qt::red),2,Qt::PenStyle(Qt::DashLine)));

            // -----------------------------------------
            m_graphScen->addRect(QRect(1,1,x.width()-4,x.height()-4),QPen(QBrush(Qt::gray),1));

            ui->graphicsView->setScene(m_graphScen);
            ui->graphicsView->show();

            if(mysocket->m_speed < 300 && mysocket->m_speed > 0)
            {
                ui->speed->setText(QString("%1").arg(abs(mysocket->m_speed), 0, 'f', 1));
            }

//#ifndef Q_OS_MAC
            if(mysocket->IMUconnected == true)
            {
                ui->roll->setText(QString("%1").arg(abs(roll_att), 0, 'f', 0));
                ui->pitch->setText(QString("%1").arg((pitch_att), 0, 'f', 0));
                ui->temp->setText(QString("%1").arg(mysocket->Temp, 0, 'f', 0));
                ui->temperature->setText(QString("%1").arg(mysocket->m_gpsspeed, 0, 'f', 1));
                ui->compass->setText(QString("%1").arg(m_head, 0, 'f', 0));
            }
            else if(m_rotation_sensor != nullptr)
//#endif
            {
                ui->roll->setText(QString("%1").arg(abs(roll_att), 0, 'f', 0));
                ui->pitch->setText(QString("%1").arg((pitch_att), 0, 'f', 0));
                ui->compass->setText(QString("%1").arg(m_head, 0, 'f', 0));
                if(this->mysocket->Temp != -100.0){
                    ui->temperature->setText(QString("%1").arg(this->mysocket->Temp, 0, 'f', 1));
                }
            }

            if( m_geopos == true)
            {
                ui->altitude->setText(QString("%1").arg(this->mysocket->m_altitude, 0, 'f', 0));
            }
        }
        m_reading|=0x04;
    }
}


void MainWindow::reset_ping()
{
    qDebug() << ":::Ping timeout";
    QString x;
    x = ui->pushButton_10->styleSheet();
    x.replace(QString("1 #090"), QString("1 #900"));
    ui->pushButton_10->setStyleSheet(x);
    ui->pushButton_10->update();
}

// Timeout on the transponder port...
void MainWindow::active_ping()
{
    QString x;
    x = ui->pushButton_14->styleSheet();
    x.replace(QString("1 #090"), QString("1 #900"));
    ui->pushButton_14->setStyleSheet(x);
    ui->pushButton_14->update();
    this->timerActive->stop();
}

void MainWindow::doCheck()
{
    if ( alt_receiced == false)
    {
        QString x = ui->pushButton_11->styleSheet();
        x.replace(QString("1 #090"), QString("1 #900"));
        ui->pushButton_11->setStyleSheet(x);
        ui->pushButton_11->update();
    }else{
        QString x = ui->pushButton_11->styleSheet();
        x.replace(QString("1 #900"), QString("1 #090"));
        ui->pushButton_11->setStyleSheet(x);
        ui->pushButton_11->update();
    }
    alt_receiced = false;
}

void MainWindow::setalt(int alt_mode)
{
    this->alt_mode = alt_mode;
    qDebug() << "Set ALT: " << alt_mode;
}

void MainWindow::getTransponderVal() //const QByteArray &array)
{
    if(this->mysocket->transponder_valid)
    {
        QMetaObject::invokeMethod(this, [this]() {
            QString x = this->ui->pushButton_14->styleSheet();
            if (x.contains("#900")) {
                x.replace("1 #900", "1 #090");
                this->ui->pushButton_14->setStyleSheet(x);
                this->ui->pushButton_14->update();
            }
            this->timerActive->start(5000);
        }, Qt::QueuedConnection);
        this->mysocket->transponder_valid = false;
    }

    if(this->mysocket->transponder_ping)
    {
        QMetaObject::invokeMethod(this, [this]() {
            QString x = this->ui->pushButton_10->styleSheet();
            x.replace(QString("1 #900"), QString("1 #090"));
            this->ui->pushButton_10->setStyleSheet(x);
            this->ui->pushButton_10->update();
            this->timerPing->start(10000);
        }, Qt::QueuedConnection);
        this->mysocket->transponder_ping = false;
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_s != '-')
    {
        auto setButtonActive = [](QPushButton *button, bool active)
        {
            QString style = button->styleSheet();
            //                                qDebug() << style;

            if (active) {
                // Only change if it is currently inactive
                if (style.contains("1 #888")) {
                    style.replace("1 #888", "1 #2A0");
                    button->setStyleSheet(style);
                    button->update();
                    QThread::msleep(100);
                }else{

                }
            } else {
                // Only change if it is currently active
                if (style.contains("1 #2A0")) {
                    style.replace("1 #2A0", "1 #888");
                    button->setStyleSheet(style);
                    button->update();
                    QThread::msleep(100);
                }
            }
        };

        int newMode = this->mode;

        switch (this->mysocket->transponder_command_s )
        {
        case 't':   newMode = 1;    break;
        case 'a':   newMode = 2;    break;
        case 'c':   newMode = 3;    break;
        default:                    break;
        }

        // Only do anything if the mode actually changed
        if (newMode != this->mode){
            setButtonActive(this->ui->pushButton_stby, newMode == 1);
            setButtonActive(this->ui->pushButton_norm, newMode == 2);
            setButtonActive(this->ui->pushButton_alt,  newMode == 3);
            this->mode = newMode;
        }
        this->mysocket->transponder_command_s = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_r != '-')
    {
        bool state=true;
        if (this->mysocket->transponder_command_r  == 'N') state = false;
        QString x = tr("Annunciator %1").arg(state);
        this->mysocket->transponder_command_r = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_i != '-')
    {
        bool state;
        if (this->mysocket->transponder_command_i  == '0') state = false;
        else state = true;

        QString x = this->ui->pushButton_Ident->styleSheet();

        if ( state == false)
        {
            x.replace(QString("1 #900"), QString("1 #888"));
        }else{
            x.replace(QString("1 #888"), QString("1 #900"));
        }
        this->ui->pushButton_Ident->setStyleSheet(x);
        this->ui->pushButton_Ident->update();

        this->mysocket->transponder_command_i = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_c[0] != '-')
    {
        int number;
        char numout[5];
        sscanf(this->mysocket->transponder_command_c,"c=%d",&number);
        snprintf(numout,5,"%.4d",number);

        this->current[3]=numout[3]-0x30;
        this->current[2]=numout[2]-0x30;
        this->current[1]=numout[1]-0x30;
        this->current[0]=numout[0]-0x30;

        this->ui->lcdNumber->display(QString::number( this->current[0]*1000+
                                                     this->current[1]*100+
                                                     this->current[2]*10+
                                                     this->current[3]).rightJustified(4, '0'));
        this->ui->lcdNumber->update();
        this->mysocket->transponder_command_c[0] = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_a[0] != '-')
    {
        float number;
        char numout[20];
        if(this->mysocket->transponder_command_a[2] == '?'){
            this->mysocket->transponder_command_a[2] = '1';
            this->mysocket->transponder_command_a[3] = 'M';
            this->mysocket->transponder_command_a[4] = 0;
        }
        sscanf(this->mysocket->transponder_command_a,"a=%f",&number);
        QString altType="Alt.Ft.";

        if(this->alt_mode == 1)
        {
            for (unsigned long key=0; key < strlen(this->mysocket->transponder_command_a); key++)
            {
                if(this->mysocket->transponder_command_a[key]=='M')
                {
                    number*=3.2808399;
                    break;
                }
            }
            this->m_tansALT = round(number/100.0)*100;
        }
        else{
            for (unsigned long key=0; key < strlen(this->mysocket->transponder_command_a); key++)
            {
                if(this->mysocket->transponder_command_a[key]=='F')
                {
                    number/=3.2808399;
                    break;
                }
            }
            this->m_tansALT = round(number);
            altType="Alt.M.";
        }
        snprintf(numout,20,"%.4d",(int)this->m_tansALT);
        this->ui->lcdNumber_3->display(numout);
        this->ui->lcdNumber_3->update();
        this->ui->label_2->setText(altType);
        this->ui->label_2->update();

//                        local_ui->baro_alt->setText(numout);

        // We assume that altitude never will is zero, this might not hold,
        // but there has been some issues with the altitude encoder so we do this any how...
        if((int)this->m_tansALT > 0)
            this->alt_receiced = true;

        this->mysocket->transponder_command_a[0] = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_z[0] != '-')
    {
        qDebug() << "T: %s\r\n" << &this->mysocket->transponder_command_z[2];
        this->ui->plainTextEdit->appendPlainText(&this->mysocket->transponder_command_z[2]);
        this->ui->plainTextEdit->update();
        this->mysocket->transponder_command_z[0] = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_p != '-')
    {
        qDebug() << "P: %c\r\n" << &this->mysocket->transponder_command_p;

        bool state=true;
        if (this->mysocket->transponder_command_p == '1') state = false;

        QString x = tr("Hardware test status: %1").arg(state);
        this->ui->plainTextEdit->appendPlainText(x);
        this->ui->plainTextEdit->update();

        // ...
        if(state == true)
        {
            x = this->ui->pushButton_10->styleSheet();
            x.replace(QString("1 #900"), QString("1 #090"));
            this->ui->pushButton_10->setStyleSheet(x);
            this->ui->pushButton_10->update();
            this->timerPing->stop();
            this->timerPing->start(5000); // Turn off in 5 sec...
        }
        else
        {
            x = this->ui->pushButton_10->styleSheet();
            x.replace(QString("1 #090"), QString("1 #900"));
            this->ui->pushButton_10->setStyleSheet(x);
            this->ui->pushButton_10->update();
        }

        this->mysocket->transponder_command_p = '-';
    }
//    QCoreApplication::processEvents();
}

void MainWindow::accepted(void)
{
    qApp->closeAllWindows();
    qApp->exit();
    QCoreApplication::quit();
    this->close();
}

void MainWindow::addnext(int x)
{
    this->next[0]=this->next[1];
    this->next[1]=this->next[2];
    this->next[2]=this->next[3];
    this->next[3]=x;

    QString num = QString::number(this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]).rightJustified(4, '0');
    ui->lcdNumber_2->display( num);
}

void MainWindow::setmode(int m)
{
    if(mysocket != NULL)
    {
        switch(m){
        case 0: mysocket->readyWrite((char*)"\x02" "s=?" "\x03"); break;
        case 1: mysocket->readyWrite((char*)"\x02" "s=t" "\x03"); break;
        case 2: mysocket->readyWrite((char*)"\x02" "s=a" "\x03"); break;
        case 3: mysocket->readyWrite((char*)"\x02" "s=c" "\x03"); break;
        }
    }
   // mode = m;
}

void MainWindow::on_pushButton_clicked(  ){addnext(1);}
void MainWindow::on_pushButton_2_clicked(){addnext(2);}
void MainWindow::on_pushButton_3_clicked(){addnext(3);}
void MainWindow::on_pushButton_4_clicked(){addnext(4);}
void MainWindow::on_pushButton_5_clicked(){addnext(5);}
void MainWindow::on_pushButton_6_clicked(){addnext(6);}
void MainWindow::on_pushButton_7_clicked(){addnext(7);}
void MainWindow::on_pushButton_8_clicked(){};//addnext(8);}
void MainWindow::on_pushButton_9_clicked(){};//addnext(9);}
void MainWindow::on_pushButton_17_clicked(){addnext(0);}

void MainWindow::on_exit_2_clicked(){           this->close();  }
void MainWindow::on_pushButton_19_clicked(){    this->close();  }

void MainWindow::on_pushButton_16_clicked()
{
    this->current[3]=this->next[3];
    this->current[2]=this->next[2];
    this->current[1]=this->next[1];
    this->current[0]=this->next[0];

    char data[100];
    snprintf(data,100,"\x02" "c=%d" "\x03",this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]);
    //    QString num = QString::number(this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]);
    //    QString msg = QString("c=%1\r\n").arg(num);
    qDebug() << data;

    mysocket->readyWrite(data);
}

void MainWindow::on_pushButton_18_clicked()
{
    this->next[3]=0;
    this->next[2]=0;
    this->next[1]=0;
    this->next[0]=7;

    QString num = QString::number(this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]).rightJustified(4, '0');
    ui->lcdNumber_2->display( num);

    char data[100];
    snprintf(data,100,"\x02" "c=%d" "\x03",7000);
    qDebug() << data;

    mysocket->readyWrite(data);
}


void MainWindow::on_pushButton_Ident_clicked()
{
    mysocket->readyWrite((char*)"\x02" "i=s" "\x03");
}

// Set mode...
void MainWindow::on_pushButton_stby_clicked(){setmode(1);}
void MainWindow::on_pushButton_norm_clicked(){setmode(2);}
void MainWindow::on_pushButton_alt_clicked(){setmode(3);}

void MainWindow::on_pushButton_12_clicked(){setalt(0);}
void MainWindow::on_pushButton_13_clicked(){setalt(1);}

void MainWindow::on_pushButton_off_clicked(){
    mysocket->readyWrite((char*)"\x02" "p=?" "\x03");
}

//-------------------------------------------------------------
//-------------------------------------------------------------
// Index 0 Transponder
void MainWindow::on_select_up_clicked()
{
    changePage(+1);
}

void MainWindow::on_select_down_clicked()

{
    changePage(-1);
}

void MainWindow::changePage(int direction)
{
    // Leave camera page
    if (currentIndex == P_CAMERA)
        hideCamera();

    currentIndex += direction;

    // Wrap around
    if (currentIndex > P_CAMERA)
        currentIndex = P_TRANSPONDER;
    else if (currentIndex < P_TRANSPONDER)
        currentIndex = P_CAMERA;

    // Skip Transponder page if unavailable
    if (currentIndex == P_TRANSPONDER && !mysocket->Transponderstat)
        currentIndex += direction;

    // Wrap again if skipping crossed the end
    if (currentIndex > P_CAMERA)
        currentIndex = P_TRANSPONDER;
    else if (currentIndex < P_TRANSPONDER)
        currentIndex = P_CAMERA;

    // Enter camera page
    if (currentIndex == P_CAMERA)
        setCamera(QMediaDevices::defaultVideoInput());

    ui->stackedWidget->setCurrentIndex(currentIndex);
}


/*
void MainWindow::on_select_up_clicked(){
    if(currentIndex == P_CAMERA)
        currentIndex = P_TRANSPONDER;
    else
        currentIndex++;

    if(currentIndex == P_TRANSPONDER)
        hideCamera();

    if( currentIndex == P_TRANSPONDER && mysocket->Transponderstat == false){
        currentIndex = P_IMU;
    }

    if(currentIndex == P_CAMERA)
        setCamera(QMediaDevices::defaultVideoInput());

    ui->stackedWidget->setCurrentIndex(currentIndex);
}
void MainWindow::on_select_down_clicked(){
    if(currentIndex == P_TRANSPONDER)
        currentIndex = P_CAMERA;
    else
        currentIndex--;

    if(currentIndex == P_CONFIG)
        hideCamera();

    if( currentIndex == P_TRANSPONDER && mysocket->Transponderstat == false){
        currentIndex = P_CAMERA;
    }

    if(currentIndex == P_CAMERA)
        setCamera(QMediaDevices::defaultVideoInput());

    ui->stackedWidget->setCurrentIndex(currentIndex);
}
*/
//-------------------------------------------------------------
//-------------------------------------------------------------

void MainWindow::on_imu_reset_clicked()
{
    m_calibrate = 100;
    m_msgBoxCalibrating = new NoButtonMessageBox(tr("Full Calibration\nPlease make sure the equipment is still !"));
    m_msgBoxCalibrating->show();
}

void MainWindow::on_use_hw_clicked()
{
    m_first = 10;

    m_msgBoxCalibrating = new NoButtonMessageBox(tr("Calibrating\nPlease make sure the equipment is still !"));
    m_msgBoxCalibrating->show();
}

void MainWindow::on_timer_start_clicked()
{
    QString x = ui->timer_start->styleSheet();
    if(m_armed == false)
    {
        m_armed = true;
        x.replace(QString("1 #888"), QString("1 #00F"));
        qDebug() << "Timer pressed 1...";
        ui->listView->setHidden(false);
    }
    else{
        m_armed = false;
        x.replace(QString("1 #00F"), QString("1 #888"));
        qDebug() << "Timer pressed 2...";
        ui->listView->setHidden(true);
    }

    ui->timer_start->setStyleSheet(x);
    ui->timer_start->update();
}


void MainWindow::on_textEdit_1_textChanged()
{
    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(RADIO));
    if( l_file->open(QIODevice::ReadWrite ))
    {
        QString data = ui->textEdit->toPlainText();
        l_file->write(data.toLocal8Bit());
        l_file->close();
    }
}


void MainWindow::on_textEdit_2_textChanged()
{
    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(AIRPLANE));
    if( l_file->open(QIODevice::ReadWrite ))
    {
        QString data = ui->textEdit_2->toPlainText();
        l_file->write(data.toLocal8Bit());
        l_file->close();
    }

}

int MainWindow::get_default_config(Matrix3x6 &sensor)
{
    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(CONFIG));
    if (l_file->open(QIODevice::ReadOnly))
    {
        QString blob = l_file->readAll();
        l_file->close();

        QStringList pieces = blob.split( "\n");
        if(pieces.length() > 1)
        {
            static int found = 0;
            for(QString value : pieces)
            {
                QStringList element = value.split(",");

                if(element[0] == "AccelCal")
                {
                    sensor[0][0] = element[2].toDouble();
                    sensor[0][1] = element[4].toDouble();
                    sensor[0][2] = element[6].toDouble();
                    sensor[0][3] = element[8].toDouble();
                    sensor[0][4] = element[10].toDouble();
                    sensor[0][5] = element[12].toDouble();
                    found++;
                }
                if(element[0] == "GyroCal")
                {
                    sensor[1][0] = element[2].toDouble();
                    sensor[1][1] = element[4].toDouble();
                    sensor[1][2] = element[6].toDouble();
                    sensor[1][3] = element[8].toDouble();
                    sensor[1][4] = element[10].toDouble();
                    sensor[1][5] = element[12].toDouble();
                    found++;
                }
                if(element[0] == "MagCal")
                {
                    sensor[2][0] = element[2].toDouble();
                    sensor[2][1] = element[4].toDouble();
                    sensor[2][2] = element[6].toDouble();
                    sensor[2][3] = element[8].toDouble();
                    sensor[2][4] = element[10].toDouble();
                    sensor[2][5] = element[12].toDouble();
                    found++;
                }
            }
            if(found == 6) return 0;
        }
        return -2;
    }
    return -1;
}

int MainWindow::set_default_config(const Matrix3x6 &sensor)
{
    QString txt = "AccelCal,X1,%1,Y1,%2,Z1,%3,X2,%4,Y2,%5,Z2,%6\n";
    txt.append(   "GyroCal,X,%7,Y,%8,Z,%9,X2,%10,Y2,%11,Z2,%12\n");
    txt.append(   "MagCal,X,%13,Y,%14,Z,%15,X2,%16,Y2,%17,Z2,%18\n");

    QString data  =  txt.arg(sensor[0][0]).arg(sensor[0][1]).arg(sensor[0][2]).arg(sensor[0][3]).arg(sensor[0][4]).arg(sensor[0][5])
                        .arg(sensor[1][0]).arg(sensor[1][1]).arg(sensor[1][2]).arg(sensor[1][3]).arg(sensor[1][4]).arg(sensor[1][5])
                       .arg(sensor[2][0]).arg(sensor[2][1]).arg(sensor[2][2]).arg(sensor[2][3]).arg(sensor[2][4]).arg(sensor[2][5]);

    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(CONFIG));
    if( l_file->open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        l_file->write(data.toLocal8Bit());
        l_file->close();
        return 0;
    }
    return -1;
}

int MainWindow::set_default_radio()
{
    ui->textEdit->setText("121.500\tEmergency\n");
    ui->textEdit->insertPlainText("123.500\tStandard\n");
    ui->textEdit->insertPlainText("122.500\tNAK\n");
    ui->textEdit->insertPlainText("123.450\tPlane to Plane\n");
    ui->textEdit->insertPlainText("119.100\tKjeller\n");
    ui->textEdit->insertPlainText("122.700\tTorsnes\n");
    ui->textEdit->insertPlainText("118.470\tOslo Approache\n");

    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(RADIO));
    qDebug() << "FILE: " << l_file->fileName();
    if( l_file->open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        QString data = ui->textEdit->toPlainText();
        qDebug() << data;
        ui->plainTextEdit->appendPlainText(data);
        l_file->write(data.toLocal8Bit());
        l_file->close();
        return 0;
    }
    return -1;
}

int MainWindow::set_default_planes()
{
    ui->textEdit_2->setText("LN-YKQ\tRans S6\t\tSam S.\n");
    ui->textEdit_2->insertPlainText("LN-YYX\tAeros 2\t\tTerje\n");
    ui->textEdit_2->insertPlainText("LN-YXY\tDelta-jet TL22\tTerje\n");
    ui->textEdit_2->insertPlainText("LN-YPE\tAir Creation\tHarald\n");
    ui->textEdit_2->insertPlainText("LN-YRI\tPeregrin__\tRingerike\n");
    ui->textEdit_2->insertPlainText("LN-YRM\tAeroprak\tRingerike\n");
    ui->textEdit_2->insertPlainText("LN-YZG\tIcarus\t\tMorten\n");
    ui->textEdit_2->insertPlainText("LN-YYV\tIcarus\t\tTerje S.\n");
    ui->textEdit_2->insertPlainText("LN-YPL\tIcarus\t\tOlaf\n");
    ui->textEdit_2->insertPlainText("LN-YOG\tSamba\t\tJan Ove\n");
    ui->textEdit_2->insertPlainText("LN-YRY\tRans S6S\tAlf Nipe\n");

    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(AIRPLANE));
    if( l_file->open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        QString data = ui->textEdit_2->toPlainText();
        qDebug() << data;
        ui->plainTextEdit->appendPlainText(data);
        l_file->write(data.toLocal8Bit());
        l_file->close();
        return 0;
    }
    return -1;
}

void MainWindow::on_pushButton_15_clicked()
{
    set_default_radio();
    set_default_planes();

}

void MainWindow::on_reconnect_now_clicked()
{
    delete(mysocket);
    mysocket = new MyTcpSocket(this, ui->plainTextEdit, &this->setIMU);
}

void MainWindow::on_pushButton_20_clicked()
{
    qDebug() << "--------Take picture --------- " ;   
    m_capture->capture();
}

void MainWindow::takePicture()
{
    on_pushButton_20_clicked();
}

void MainWindow::on_pushButton_21_clicked()
{
    timertakePicture->start(3000);
}

void MainWindow::on_pushButton_22_clicked()
{
    timertakePicture->start(10000);
}

void MainWindow::on_pushButton_23_clicked()
{
    timertakePicture->stop();
}

void MainWindow::on_reset_heading_clicked()
{
    heading_offset = m_head; //mysocket->AngleZ;

    //    heading_offset = 180.0 - mysocket->AngleZ;
    //    if(heading_offset<0.0) heading_offset+=360.0;

    if(m_compass_sensor){
        //      m_compass_reader->setAzimuth(heading_offset);
    }
}

void MainWindow::on_reset_att_clicked()
{
    qDebug() << "Use IMU clicked... " << ekf.m_use_gpt;

    QString x = ui->reset_att->styleSheet();
    qDebug() << x;

    if(ekf.m_use_gpt == 0)
    {
        ekf.m_use_gpt = 1;
        m_rotation_sensor->setDataRate(4);
        x.replace(QString("1 #f00"), QString("1 #00F"));
        ui->reset_att->setText("EKF_1");
    }
    else if(ekf.m_use_gpt == 1)
    {
        if(m_use_imu) ekf.m_use_gpt = 2;
        else ekf.m_use_gpt = 3;
        m_rotation_sensor->setDataRate(4);
        x.replace(QString("1 #00F"), QString("1 #0F0"));
        ui->reset_att->setText("EKF_2");
    }
    else if(ekf.m_use_gpt == 2)
    {
        ekf.m_use_gpt = 3;
        m_rotation_sensor->setDataRate(4);
        x.replace(QString("1 #00F"), QString("1 #0F0"));
        ui->reset_att->setText("EXT.");
    }
    else{
        ekf.m_use_gpt = 0;
        m_rotation_sensor->setDataRate(50);
        x.replace(QString("1 #0F0"), QString("1 #f00"));
        ui->reset_att->setText("INT.");

    }
    qDebug() << x;
    ui->reset_att->setStyleSheet(x);
    ui->reset_att->update();
}

void MainWindow::on_fly_home_clicked()
{
    static bool pingpong=true;

//    qDebug() << "FlyHome... ";
  //  ui->quickWidget->rootObject()->setProperty("zoomLevel", 5); // 18);

    if(pingpong){
        mysocket->mqtt->sendMessage("xplane/engage", "Activate");
        ui->Autopilot_status->setText("Activated");

//        ui->quickWidget->raise();
        ui->fly_home->raise();
 //       ui->select_transponder_page2_2->raise();
 //       ui->select_gyro_page2_2->raise();
    }else{
        mysocket->mqtt->sendMessage("xplane/engage", "Deactivate");
        ui->Autopilot_status->setText("Deactivated");

 //       ui->graphicsView_3->raise();
        ui->fly_home->raise();
//        ui->select_transponder_page2_2->raise();
//        ui->select_gyro_page2_2->raise();
    }
    pingpong= !pingpong;
}

void MainWindow::on_dial_valueChanged(int qnh)
{
    ui->doubleSpinBox->setText(QString("%1").arg(qnh/100.0));
    ui->doubleSpinBox_2->setText(QString("%1").arg(qnh/100.0));
    ui->dial_2->setSliderPosition(qnh);
    _widgetALT->setPressure((float)qnh/100.0);
    _widgetALT->redraw();

}

void MainWindow::on_dial_2_valueChanged(int qnh)
{
    ui->doubleSpinBox_2->setText(QString("%1").arg(qnh/100.0));
    ui->doubleSpinBox->setText(QString("%1").arg(qnh/100.0));
    ui->dial->setSliderPosition(qnh);
    _widgetALT->setPressure((float)qnh/100.0);
    _widgetALT->redraw();
}

void MainWindow::on_use_gps_in_attitude_clicked()
{
    static bool pressed = false;

    if(pressed)
    {
        pressed = false;
        m_use_gps_in_attitude = false;
    }
    else{
        pressed = true;
        m_use_gps_in_attitude = true;
    }
}

void MainWindow::on_reset_altitude_2_clicked()
{
    double pressure_mb = setQNH()*100.0;
    on_dial_valueChanged(pressure_mb);
    qDebug() << "Altitutde. " << pressure_mb;
}

void MainWindow::on_reset_altitude_3_clicked()
{
    if(m_takeoff == false) on_reset_altitude_2_clicked();
    qDebug() << "Alt 2. ";
}

void MainWindow::on_reset_heading_2_clicked()
{
    qDebug() << "Heading. ";

}

void MainWindow::on_use_ins_only_clicked()
{
    QString x = ui->use_ins_only->styleSheet();

    this->mysocket->use_ins_only = !this->mysocket->use_ins_only;
    if(this->mysocket->use_ins_only == true)
    {
        x.replace(QString("1 #080"), QString("1 #800"));
        ui->use_built_inn_barometer->setText("Using any sensor...");

    }
    else{
        x.replace(QString("1 #800"), QString("1 #080"));
        ui->use_built_inn_barometer->setText("Using INS only...");
    }
    ui->use_ins_only->setStyleSheet(x);
    ui->use_ins_only->update();
}

void MainWindow::on_use_built_inn_barometer_clicked()
{
    static bool active = false;
    QString x = ui->use_built_inn_barometer->styleSheet();

    if(mysocket->m_pressure_raw > 1.0 ){
        if(active == false)
        {
            active = true;
            mysocket->TransponderstatWithBarometer = true;
            x.replace(QString("1 #080"), QString("1 #800"));
            ui->use_built_inn_barometer->setText("Use Built In\nbarometer");

            if( mysocket->Transponderstat == true)
            {
                mysocket->readyWrite((char*)"\x02" "d=g" "\x03");
//                mysocket->readyWrite((char*)"\x02" "d=s" "\x03");
            }

        }
        else{
            active = false;
            mysocket->TransponderstatWithBarometer = false;
            x.replace(QString("1 #800"), QString("1 #080"));
            ui->use_built_inn_barometer->setText("Use External\nbarometer");

            if( mysocket->Transponderstat == true)
            {
                mysocket->readyWrite((char*)"\x02" "d=g" "\x03");
            }
        }

        ui->use_built_inn_barometer->setStyleSheet(x);
        ui->use_built_inn_barometer->update();
    }
}
