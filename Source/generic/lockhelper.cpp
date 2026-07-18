#include <QDebug>
#include <QJniObject>
//#include <QNativeInterface>

#include "lockhelper.h"

KeepAwakeHelper::KeepAwakeHelper()
{
}

void KeepAwakeHelper::EnableKeepAwakeHelper()
{
    QNativeInterface::QAndroidApplication::runOnAndroidMainThread([]() {
        QJniObject activity =
            QNativeInterface::QAndroidApplication::context();

        if (!activity.isValid()) {
            qWarning() << "KeepAwake: Android activity is invalid";
            return QVariant();
        }

        QJniObject window = activity.callObjectMethod(
            "getWindow",
            "()Landroid/view/Window;"
            );

        if (!window.isValid()) {
            qWarning() << "KeepAwake: Android window is invalid";
            return QVariant();
        }

        constexpr jint FLAG_KEEP_SCREEN_ON = 0x00000080;

        window.callMethod<void>(
            "addFlags",
            "(I)V",
            FLAG_KEEP_SCREEN_ON
            );

        qDebug() << "KeepAwake: screen will remain on";
        return QVariant();
    });
}

void KeepAwakeHelper::DisableKeepAwakeHelper()
{
    QNativeInterface::QAndroidApplication::runOnAndroidMainThread([]() {
        QJniObject activity =
            QNativeInterface::QAndroidApplication::context();

        if (!activity.isValid()) {
            return QVariant();
        }

        QJniObject window = activity.callObjectMethod(
            "getWindow",
            "()Landroid/view/Window;"
            );

        if (!window.isValid()) {
            return QVariant();
        }

        constexpr jint FLAG_KEEP_SCREEN_ON = 0x00000080;

        window.callMethod<void>(
            "clearFlags",
            "(I)V",
            FLAG_KEEP_SCREEN_ON
            );

        qDebug() << "KeepAwake: normal screen timeout restored";
        return QVariant();
    });
}