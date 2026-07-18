// comqt.cpp
#include <QCoreApplication>
#include <QDebug>
#include <QTimer>
#include <QThread>
#include <QByteArray>
#include <QMetaObject>

#include <QJniObject>
#include <QJniEnvironment>

// If you're on Qt 6, this is enough. Avoid using private androidextras header.
// #include <QtCore/private/qandroidextras_p.h>  // <- not needed for Qt 6

#include "serialport.h"   // fixed: removed stray .h

extern "C" {

// Helper: fetch nativeHandle from 'thiz'
static ComQt* getSelf(JNIEnv* env, jobject thiz) {
    jclass cls = env->GetObjectClass(thiz);
    jfieldID fid = env->GetFieldID(cls, "nativeHandle", "J");
    if (!fid) return nullptr;
    jlong ptr = env->GetLongField(thiz, fid);
    return reinterpret_cast<ComQt*>(ptr);
}

// void nativeOnSerialBytes(byte[] data)
JNIEXPORT void JNICALL
Java_com_hoho_android_usbserial_driver_TestClassTerje_nativeOnSerialBytes(
    JNIEnv* env, jobject thiz, jbyteArray jdata)
{
    ComQt* self = getSelf(env, thiz);
    if (!self || !jdata) return;

    const jsize n = env->GetArrayLength(jdata);
    if (n <= 0) return;

    QByteArray bytes;
    bytes.resize(n);
    env->GetByteArrayRegion(jdata, 0, n, reinterpret_cast<jbyte*>(bytes.data()));

    if (self->callback_) {
        self->callback_(self->parent, bytes,bytes.length());
    }

/*
    connect(self, &ComQt::dataReceived,
            this, [](const QByteArray &bytes) {
                qDebug() << "Received:" << bytes;
                qDebug() << "As text:" << QString::fromLatin1(bytes);
            });
    emit self->dataReceived(bytes);
*/
/*
    QMetaObject::invokeMethod(self, [self, b = std::move(bytes)]() {
        qDebug() << "received::::0";
        if (self->callback_) {
            qDebug() << "received::::1";
            self->callback_(self->parent, b.constData(),
                            static_cast<uint32_t>(b.size()));
        }
        emit self->dataReceived(b);
    }, Qt::QueuedConnection);
*/
}

// void nativeOnConnected(boolean connected)
JNIEXPORT void JNICALL
Java_com_hoho_android_usbserial_driver_TestClassTerje_nativeOnConnected(
    JNIEnv* env, jobject thiz, jboolean jconnected)
{
    ComQt* self = getSelf(env, thiz);
    if (!self) return;
    const bool connected = (jconnected == JNI_TRUE);

    QMetaObject::invokeMethod(self, [self, connected]() {
        emit self->connectionChanged(connected);
    }, Qt::QueuedConnection);
}

// void nativeOnSerialError(String message)
JNIEXPORT void JNICALL
Java_com_hoho_android_usbserial_driver_TestClassTerje_nativeOnSerialError(
    JNIEnv* env, jobject thiz, jstring jmsg)
{
    ComQt* self = getSelf(env, thiz);
    if (!self) return;

    QJniObject jm(jmsg);
    const QString msg = jm.toString();

    QMetaObject::invokeMethod(self, [self, msg]() {
        qDebug() << "ERROR:::::::" << msg;
        emit self->errorReceived(msg);
    }, Qt::QueuedConnection);
}

} // extern "C"

// ===================== ComQt implementation =====================

ComQt::ComQt(QObject* parent_)
    : QObject(parent_)
{
    this->parent = parent_;
    g_comqt = this; // make this instance available to JNI callbacks
}

ComQt::~ComQt()
{
    close();
    if (g_comqt == this) g_comqt = nullptr;
}

bool ComQt::open(const QString& portName, qint32 baudrate)
{
    // Obtain Android Context
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QJniObject context = QJniObject(QNativeInterface::QAndroidApplication::context());
#else
    QJniObject context = QJniObject::callStaticObjectMethod(
        "org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
#endif

    if (!QJniObject::isClassAvailable("com/hoho/android/usbserial/driver/TestClassTerje")) {
        qWarning() << "Java class not available!";
        close();
        return false;
    }

    someJavaObject = new QJniObject(
        "com/hoho/android/usbserial/driver/TestClassTerje",
        "(Landroid/content/Context;)V",
        context.object());

    if (!someJavaObject || !someJavaObject->isValid()) {
        qWarning() << "Failed to create Java TestClassTerje instance";
        close();
        return false;
    }

    // someJavaObject is the *instance* of TestClassTerje you created
    jlong self = static_cast<jlong>(reinterpret_cast<intptr_t>(this));
    someJavaObject->callMethod<void>("setNativeHandle", "(J)V", self);

    // Call: String connectserial(String serial, int baudrate)
    QString stat = "";
    const QJniObject jSerial = QJniObject::fromString(portName);
    const QString status = someJavaObject->callMethod<jstring>(
        "connectserial",
        "(Ljava/lang/String;I)Ljava/lang/String;",
        jSerial.object(),
        jint(baudrate)).toString();
    stat = status;

    qDebug() << "connect-serial status:" << stat;

    if (stat == QLatin1String("-5")) {
        QString usb = "false";
        while (usb != QLatin1String("true")) {
            const QString usblegal = someJavaObject->callMethod<jstring>(
                                                      "usbpremission").toString();
            usb = usblegal;
            QThread::msleep(100);  // Wait 100 milliseconds
            qDebug() << "USB-status:" << usblegal;
        }

        const QString statusx = someJavaObject->callMethod<jstring>(
                                                  "connectserial",
                                                  "(Ljava/lang/String;I)Ljava/lang/String;",
                                                  jSerial.object(),
                                                  jint(baudrate)
                                                  ).toString();

        stat = statusx;
        qDebug() << "connect-serial status:" << statusx;
    }

    if (stat == QLatin1String("1") || stat == QLatin1String("0")) {
        // Get device info
        const QString serialnumber = someJavaObject->callObjectMethod(
                                                       "getInfo", "()Ljava/lang/String;").toString();
        qDebug() << "SerialNumber:" << serialnumber;

        // Set control lines (our Java side expects 1 boolean: RTS)
        someJavaObject->callMethod<jint>("ControlLines", "(Z)I", jboolean(false));

        qDebug() << "Serialport opened:" << portName;
        return true;
    }

    qWarning() << "Serialport NOT found/opened:" << portName;
 //   close();
    return false;
}

void ComQt::close()
{
    if (someJavaObject && someJavaObject->isValid()) {
        // String disconn()
        QString result = someJavaObject->callObjectMethod("disconn", "()Ljava/lang/String;").toString();
        qDebug() << "disconn:" << result;
    }
    delete someJavaObject;
    someJavaObject = nullptr;
}

bool ComQt::setBaudrate(qint32 /*baudrate*/)
{
    // Not implemented on Java side here; add a Java method if needed.
    return false;
}

bool ComQt::send(const QByteArray& data)
{
    if (!someJavaObject || !someJavaObject->isValid())
        return false;

    QJniEnvironment env;
    jbyteArray jba = env->NewByteArray(data.size());
    if (!jba) return false;

    env->SetByteArrayRegion(jba, 0, data.size(),
                            reinterpret_cast<const jbyte*>(data.constData()));

    // int sendToSerial(byte[] data)
    const jint result = someJavaObject->callMethod<jint>(
        "sendToSerial", "([B)I", jba);

    env->DeleteLocalRef(jba);

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        return false;
    }
    return (result >= 0);
}

bool ComQt::send(const char* data, unsigned short len)
{
    // fromRawData does not take ownership; copy to a real QByteArray for safety
    return send(QByteArray(data, static_cast<int>(len)));
}

// OPTIONAL legacy poller — if you keep timerAndroid running.
// Pulls a raw byte[] from Java: byte[] recFromSerial()
void ComQt::handleReadyRead(){}

