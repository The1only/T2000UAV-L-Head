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

    //Qt::ScreenOrientation ScreenMode;
    QApplication app(argc, argv);

    MainWindow d;

#if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
    d.showMaximized();
#else
    d.show();
#endif
    app.exec();

    return 0;
}



