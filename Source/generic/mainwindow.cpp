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
//#include <deque>
#include <QThread>

//***C++11 Style:***
//#include <chrono>

#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>

#include "mainwindow.h"
#include "mytcpsocket.h"

#ifdef Q_OS_ANDROID
#include "sharedstorage.h"
#endif

using namespace std;
using namespace std::chrono;

// ----------------------------------------------
// ----------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
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
    // Normal filesystem path on macOS and iOS
    const QString documentsPath = LOG_DIR;
    qDebug() << "Log directory:" << documentsPath;
    QDir dir(documentsPath);
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            qWarning() << "Could not create log directory:"
                       << documentsPath;
        }
    }

#elif defined(Q_OS_ANDROID)
/*
     * Do not create /storage/emulated/0/Download/... using QDir.
     *
     * SharedStorage.java creates the public directory automatically
     * through MediaStore when the first file is written.
     */

qDebug() << "Android logs will be stored in:"
         << "Download/LowEnergyScanner/";
#endif

    ui->setupUi(this);

// The splash screen does not make sense on a PC... but it works if you need it...
//#if defined(Q_OS_IOS) || defined(Q_OS_MACOS)
#if defined(Q_OS_IOS)
    QPixmap splashPixmap(":/images/splash.png");  // Or use a file path
    splash = new QSplashScreen(splashPixmap);
    splash->autoFillBackground();
//    splash->showMessage("Initializing Flight IMU...", Qt::AlignTop | Qt::AlignCenter, Qt::black);
    splash->show();
#endif

    // --------------------------

   // ui->quickWidget->setSource(QUrl("qrc:/places_map.qml"));
   // ui->quickWidget->rootObject()->setProperty("zoomLevel", 35); // 18);

    ui->plainTextEdit->document()->setMaximumBlockCount(50);
    ui->lcdNumber->display(QString::number(this->current[0]*1000+this->current[1]*100+this->current[2]*10+this->current[3]).rightJustified(4, '0'));
    ui->lcdNumber_2->display(QString::number(this->next[0]*1000+this->next[1]*100+this->next[2]*10+this->next[3]).rightJustified(4, '0'));
    ui->plainTextEdit->appendPlainText("Glasscockpit 200-UAV v1.02a");

    // This code must be rewritten as it is depending on timeing and speed.
    // The serial ports shuld be a parameter to the constructor...
    // The m_calibrate shuld be set in a different manner...
    // Remember that the MyTcpSocket spawns a slower process only...
    this->mysocket = new MyTcpSocket(this, ui->plainTextEdit);

    // ------------------------------
    // Set STBY mode on transponder...
    setmode(1);
//    m_timer.start();

    init();

    QString data = "System booted at: "+QDateTime::currentDateTime().toString();

#ifdef Q_OS_ANDROID
    const bool success = SharedStorage::appendTextFile("LowEnergyScanner","flightlog.txt",data);
    if (!success) {
        qWarning() << "Could not write transponder log";
    }
    else{
        qWarning() << QString(FLIGHTLOG) << " at " << "LowEnergyScanner";
    }

#else
    QFile *l_file = new QFile(QString(LOG_DIR)+ QString(FLIGHTLOG));
    if( l_file->open(QIODevice::ReadWrite | QIODevice::Append ))
    {
        l_file->write(data.toLocal8Bit()+"\n");
        l_file->close();
    }
    else{
        qDebug() << "Log file error...  ";
    }
#endif

    m_msgBox = new NoButtonMessageBox(tr("Please wait for the system to boot!"));
 //   m_msgBox->show();

    // Start the clock....
    qDebug() << "  Transponder update...  ";
    m_Transponder = new QTimer(this);
    m_Transponder->setSingleShot(false);
    connect(m_Transponder, SIGNAL(timeout()), this, SLOT(getTransponderVal()));
    m_Transponder->start(250);
}

MainWindow::~MainWindow()
{
    qDebug() << "Exiting...";
    qApp->closeAllWindows();
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

// Call back funtion from the sensor handler...
void MainWindow::init()
{
    QList<QSensor*> mySensorList;

    for (const QByteArray &type : QSensor::sensorTypes())
    {
        qDebug() << "Found a sensor type:" << type;

        for (const QByteArray &identifier : QSensor::sensorsForType(type))
        {
            qDebug() << "    " << "Found a sensor of that type:" << identifier;
            QSensor* sensor = new QSensor(type, this);
            sensor->setIdentifier(identifier);
            mySensorList.append(sensor);
        }

        if(!strncmp(type,"QPressureSensor",strlen("QPressureSensor")))
        {
            this->m_pressure_sensor = new QPressureSensor();
            connect(this->m_pressure_sensor, SIGNAL(readingChanged()), this, SLOT(onPressureReadingChanged()));
            this->m_pressure_sensor->start();
            this->m_pressure_sensor->setDataRate(4);
            qDebug() << "Found a sensor QPressureSensor";
        }
    }

    qDebug() << mySensorList;

    qDebug() << "  timerPing  ";
    timerPing = new QTimer(this);
    timerPing->setSingleShot(true);
    connect(timerPing, SIGNAL(timeout()), this, SLOT(reset_ping()));

    qDebug() << "  timerActive ";
    timerActive = new QTimer(this);
    timerActive->setSingleShot(true);
    connect(timerActive, SIGNAL(timeout()), this, SLOT(active_ping()));

    qDebug() << "  timerAlt  ";
    timerAlt = new QTimer(this);
    timerAlt->setSingleShot(false);
    connect(timerAlt, SIGNAL(timeout()), this, SLOT(doCheck()));
    timerAlt->start(5000);

    QCoreApplication::processEvents();
}

void MainWindow::onPressureReadingChanged()
{
    if(m_pressure_sensor != nullptr && mysocket->Transponder_altitude_mode == 3){
        m_pressure_reader = m_pressure_sensor->reading();
        float m_pressure_raw =  m_pressure_reader->pressure()/100.0;
        // Meters...
        this->mysocket->m_altitude = 44330.0 * (1.0 - std::pow(m_pressure_raw / 1013.25, 0.190284));
        // feet...
       // this->mysocket->m_preasure_alt = 145366.45 * (1.0 - std::pow(m_pressure_raw / 1013.25, 0.190284));

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
        float number = 0.0;
        float meters = 0.0;
        char numout[20] = {0};
        QString altType;

        if(this->mysocket->transponder_command_a[2] == '?'){
            this->mysocket->transponder_command_a[2] = '1';
            this->mysocket->transponder_command_a[3] = 'M';
            this->mysocket->transponder_command_a[4] = 0;
        }
        sscanf(this->mysocket->transponder_command_a, "a=%f", &number);
        const bool reportedMeters = strchr(this->mysocket->transponder_command_a, 'M');
        const bool reportedFeet = strchr(this->mysocket->transponder_command_a, 'F');

        // First convert reported altitude to meters
        meters = number;
        if (reportedFeet) meters /= 3.2808399;

        // Then convert to selected display unit
        if (this->alt_mode == 1)   // Feet
        {
            number = meters * 3.2808399;
            number = std::round(number / 100.0) * 100.0;
            altType = "Alt.Ft.";
        }
        else                       // Meters
        {
            number = meters;
            altType = "Alt.M.";
        }

        this->m_tansALT = std::round(number);

        // If we are in AUTO mode, and the internal altimeter reports 0, then switch to the external if it exists...
        if( (meters < 0.1 || meters > 5000 ) &&
          this->mysocket->Transponder_altitude_mode == 2 &&
          this->mysocket->m_altitude > 0)
        {
            // If we got an external sensor...
            if(mysocket->Altimeter_data.altitude > 0.1){
                mysocket->TransponderMode(false);
                mysocket->Transponder_altitude_mode = 1;
                ui->pushButton_27->setText("EXT");
            }
            else if(m_pressure_sensor != nullptr){
                mysocket->TransponderMode(false);
                mysocket->Transponder_altitude_mode = 3;
                ui->pushButton_27->setText("INT");
            }

            this->m_tansALT = this->mysocket->m_altitude ;
            this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: orange; }");

        }
        snprintf(numout,20,"%.4d",(int)this->m_tansALT);
        this->ui->lcdNumber_3->display(numout);
        this->ui->lcdNumber_3->update();
        this->ui->plainTextEdit->appendPlainText(QString("Altitude to display: ") + numout);
        this->ui->label_2->setText(altType);
        this->ui->label_2->update();
//      this->ui->baro_alt->setText(numout);

        // We assume that altitude never will is zero, this might not hold,
        // but there has been some issues with the altitude encoder so we do this any how...
        if((int)this->m_tansALT > 0)
            this->alt_receiced = true;

        this->mysocket->transponder_command_a[0] = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_z[0] != '-')
    {
//        qDebug() << "T: %s\r\n" << &this->mysocket->transponder_command_z[2];
        this->ui->plainTextEdit->appendPlainText(&this->mysocket->transponder_command_z[2]);
        this->ui->plainTextEdit->update();
        this->mysocket->transponder_command_z[0] = '-';
    }

    /// ----------------------------------------------------
    if(this->mysocket->transponder_command_p != '-')
    {
//        qDebug() << "P: %c\r\n" << &this->mysocket->transponder_command_p;

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

void MainWindow::on_pushButton_27_clicked()
{

    QString text = ui->pushButton_27->text();
    if(text == "TRA"){
        ui->pushButton_27->setText("EXT");
        qDebug() << "Alt Mode: " << "EXT";

        // If we have received something from the external sensor...
        if(mysocket->Altimeter_data.altitude > 0.1){
            mysocket->Transponder_altitude_mode = 1;
            mysocket->TransponderMode(false);
            this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: orange; }");
        }
        // If we got no external sensor...
        else{
            mysocket->Transponder_altitude_mode = 0;
            mysocket->TransponderMode(true);
            this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: red; }");

        }
    }
    else if(text == "EXT"){
        ui->pushButton_27->setText("INT");
        qDebug() << "Alt Mode: " << "INT";
        if(m_pressure_sensor != nullptr){
            mysocket->Transponder_altitude_mode = 3;
            mysocket->TransponderMode(false);
            this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: #FF00FF; }");
        }
        else{
            mysocket->Transponder_altitude_mode = 0;
            mysocket->TransponderMode(true);
            this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: red; }");
        }
    }

    else if(text == "INT"){
        ui->pushButton_27->setText("AUTO");
        qDebug() << "Alt Mode: " << "AUTO";

        if(mysocket->Altimeter_data.altitude > 0.1 || m_pressure_sensor != nullptr){
            mysocket->Transponder_altitude_mode = 2;
            mysocket->TransponderMode(true);
        }
        else{
            mysocket->Transponder_altitude_mode = 0;
            mysocket->TransponderMode(true);
        }
        this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: rgb(13, 255, 252); }");
    }
    else if(text == "AUTO"){
        ui->pushButton_27->setText("TRA");
        qDebug() << "Alt Mode: " << "TRA";
        mysocket->Transponder_altitude_mode = 0;
        mysocket->TransponderMode(true);
        this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: rgb(13, 255, 252); }");
    }
    else{
        text = "TRA";
        qDebug() << "Alt Mode: " << "TRA";
        mysocket->Transponder_altitude_mode = 0;
        qDebug() << "This should not happen... #001";
        this->ui->lcdNumber_3->setStyleSheet("QLCDNumber { color: rgb(13, 255, 252); }");
    }
/*

    dTODO set Serial mode on Transponder, if connected,
        and start transmitting altitude from the local sensor...
*/
//    this->alt_mode = alt_mode;
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

void MainWindow::on_reconnect_now_clicked()
{
    delete(mysocket);
    mysocket = new MyTcpSocket(this, ui->plainTextEdit);
}

