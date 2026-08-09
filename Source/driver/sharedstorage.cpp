#include "sharedstorage.h"

#include <QDebug>

#ifdef Q_OS_ANDROID
#include <QCoreApplication>
#include <QJniEnvironment>
#include <QJniObject>
#endif

bool SharedStorage::appendTextFile(
    const QString &directory,
    const QString &fileName,
    const QString &text)
{
#ifdef Q_OS_ANDROID

    const QJniObject context =
        QNativeInterface::QAndroidApplication::context();

    if (!context.isValid()) {
        qWarning() << "SharedStorage: Android context is invalid";
        return false;
    }

    const QJniObject javaDirectory =
        QJniObject::fromString(directory);

    const QJniObject javaFileName =
        QJniObject::fromString(fileName);

    const QJniObject javaText =
        QJniObject::fromString(text);

    const jboolean result =
        QJniObject::callStaticMethod<jboolean>(
            "com/hoho/android/usbserial/util/SharedStorage",
            "appendTextFile",
            "(Landroid/content/Context;"
            "Ljava/lang/String;"
            "Ljava/lang/String;"
            "Ljava/lang/String;)Z",
            context.object<jobject>(),
            javaDirectory.object<jstring>(),
            javaFileName.object<jstring>(),
            javaText.object<jstring>());

    QJniEnvironment environment;

    if (environment.checkAndClearExceptions()) {
        qWarning() << "SharedStorage: Java exception in appendTextFile()";
        return false;
    }

    return result == JNI_TRUE;

#else

    Q_UNUSED(directory)
    Q_UNUSED(fileName)
    Q_UNUSED(text)

    qWarning() << "SharedStorage is only implemented for Android";
    return false;

#endif
}

bool SharedStorage::writeTextFile(
    const QString &directory,
    const QString &fileName,
    const QString &text)
{
#ifdef Q_OS_ANDROID

    const QJniObject context =
        QNativeInterface::QAndroidApplication::context();

    if (!context.isValid()) {
        qWarning() << "SharedStorage: Android context is invalid";
        return false;
    }

    const QJniObject javaDirectory =
        QJniObject::fromString(directory);

    const QJniObject javaFileName =
        QJniObject::fromString(fileName);

    const QJniObject javaText =
        QJniObject::fromString(text);

    const jboolean result =
        QJniObject::callStaticMethod<jboolean>(
            "com/hoho/android/usbserial/util/SharedStorage",
            "writeTextFile",
            "(Landroid/content/Context;"
            "Ljava/lang/String;"
            "Ljava/lang/String;"
            "Ljava/lang/String;)Z",
            context.object<jobject>(),
            javaDirectory.object<jstring>(),
            javaFileName.object<jstring>(),
            javaText.object<jstring>());

    QJniEnvironment environment;

    if (environment.checkAndClearExceptions()) {
        qWarning() << "SharedStorage: Java exception in writeTextFile()";
        return false;
    }

    return result == JNI_TRUE;

#else

    Q_UNUSED(directory)
    Q_UNUSED(fileName)
    Q_UNUSED(text)

    qWarning() << "SharedStorage is only implemented for Android";
    return false;

#endif
}