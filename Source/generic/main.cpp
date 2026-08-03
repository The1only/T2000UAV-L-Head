/**
 * @file main.cpp
 * @brief Implementation of main.
 *
 * Contains the implementation details for the main.
 */

#include <QtWidgets/qapplication.h>
#include <QtCore/QLoggingCategory>
#include <QApplication>
#include <QSplashScreen>
#include <QThread>
#include <QDebug>

#include "mainwindow.h"
#include "myNativeWrapperFunctions.h"

#ifdef Q_OS_ANDROID
// Enable Android keep-awake helper (no-op on other platforms)
 #define USE_KeepAwakeHelper
 #include "lockhelper.h"
#endif


#ifdef Q_OS_ANDROID

enum class AndroidOrientation {
    Unspecified     = -1,
    Landscape       = 0,
    Portrait        = 1,
    Sensor          = 4,
    SensorLandscape = 6,
    SensorPortrait  = 7,
    ReverseLandscape = 8,
    ReversePortrait  = 9,
    FullSensor      = 10,
    Locked          = 14
};

void setAndroidOrientation(AndroidOrientation orientation)
{
    QNativeInterface::QAndroidApplication::runOnAndroidMainThread(
        [orientation]() {
            QJniObject activity =
                QNativeInterface::QAndroidApplication::context();

            if (!activity.isValid()) {
                qWarning() << "Cannot set orientation: invalid Android activity";
                return QVariant();
            }

            activity.callMethod<void>(
                "setRequestedOrientation",
                "(I)V",
                static_cast<jint>(orientation));

            return QVariant();
        });
}

#endif

void qtMessageHandler(QtMsgType type,
                      const QMessageLogContext &context,
                      const QString &msg)
{
    if (msg.contains("QObject::startTimer")) {
        qDebug() << "CAUGHT TIMER WARNING:";
        qDebug() << msg;
        qDebug() << "File:" << context.file;
        qDebug() << "Line:" << context.line;
        qDebug() << "Function:" << context.function;
        qDebug() << "Current thread:" << QThread::currentThread();
        // Put a breakpoint on this line
        __builtin_trap();   // Linux/macOS GCC/Clang
    }
    fprintf(stderr, "%s\n", msg.toLocal8Bit().constData());
}

int main(int argc, char *argv[])
{
    qInstallMessageHandler(qtMessageHandler);

    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    QApplication app(argc, argv);

#ifdef Q_OS_ANDROID
#ifdef LANDSCAPE
    setAndroidOrientation(AndroidOrientation::Landscape);
#else
    setAndroidOrientation(AndroidOrientation::Portrait);
#endif
#endif
    MainWindow d;

#if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
 #ifdef Q_OS_ANDROID
    KeepAwakeHelper::EnableKeepAwakeHelper();
    QTimer::singleShot(300, &d, [&d]() {
        d.showMaximized();
        d.updateGeometry();
        d.update();
    });
 #else
   d.showMaximized();
 #endif
#else
    d.show();
#endif

    app.exec();

    return 0;
}



