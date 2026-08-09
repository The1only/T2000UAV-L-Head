# platform/android.pri
# Android-specific configuration
# ~/Library/Android/sdk/ndk/27.2.12479018/build/cmake/android.toolchain.cmake
#
# We want altitude relative to WGS-84 ellipsoid, optionally corrected by a local N offset,
#mkdir -p build-arm64 && cd build-arm64
#export NDK=~/Library/Android/sdk/ndk/27.2.12479018
#cmake ../ \
# -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
# -DANDROID_ABI=arm64-v8a \
# -DANDROID_PLATFORM=android-21 \
# -DCMAKE_BUILD_TYPE=Release \
# -DBUILD_SHARED_LIBS=ON \
# -DCMAKE_INSTALL_PREFIX=$(pwd)/install
#
#cmake --build . --target install -- -j$(nproc)

android {
    message(This is Android....)

    QT += serialport multimedia core-private core
    QT += network sensors bluetooth positioning

    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/../android
    RESOURCES += android/AndroidManifest.xml

    # for deployment: include .so into APK
    ANDROID_EXTRA_LIBS += $$PWD/../GeographicLib/build-arm64/install/lib/libGeographicLib.so
    # link path for qmake when building
    LIBS += -L$$PWD/../GeographicLib/build-arm64/install/lib -lGeographicLib
    INCLUDEPATH += $$PWD/../GeographicLib/build-arm64/install/include

    DEFINES += USE_GEOGRAPHICLIB

    SOURCES += \
        ./Source/generic/main.cpp \
        ./Source/generic/mainwindow.cpp \
        ./Source/generic/mytcpsocket.cpp \
        ./Source/generic/ssdp.cpp\
        ./Source/generic/lockhelper.cpp \
        ./Source/generic/tcpclient.cpp \
        ./Source/driver/rotation_matrix.cpp \
        ./Source/driver/gpx_parse.cpp \
        ./Source/driver/mqttclient.cpp \
        ./Source/driver/serialport_android.cpp \
        ./Source/generic/ins_driver.cpp \
        ./Source/driver/wit_c_sdk.c \
        ./Source/driver/geoid_helper.cpp \
        ./EKF_IMU_GPS/ekf_nav_ins/src/ekfNavINS.cpp \
        ./Source/driver/bleuart.cpp \
        ./Source/driver/sharedstorage.cpp \

#       ./Source/generic/multicastlistner.cpp \


    HEADERS += \
        ./Header/generic/mainwindow.h \
        ./Header/generic/mytcpsocket.h \
        ./Header/generic/lockhelper.h \
        ./Header/generic/reg.h \
        ./Header/generic/ssdp.h\
        ./Header/generic/tcpclient.h \
        ./EKF_IMU_GPS/ekf_nav_ins/inc/ekfNavINS.h \
        ./EKF_IMU_GPS/ekf_nav_ins/inc/ekfNavINS_quart.h \
        ./Header/driver/rotation_matrix.h \
        ./Header/driver/gpx_parse.h \
        ./Header/driver/mqttclient.h \
        ./Header/driver/serialport.h \
        ./Header/driver/geoid_helper.h \
       ./Header/driver/bleuart.h \
       ./Header/driver/sharedstorage.h \


#       ./Header/generic/QuickWidget.h \
#       ./Header/generic/multicastlistner.h \

    # Paho for Android (your original paths)
    PAHO_ROOT = /Users/terjenilsen/Dropbox/Sportsfly/transponder/third_party/paho/$$ANDROID_TARGET_ARCH
    INCLUDEPATH += $$PAHO_ROOT/include
    LIBS += -L$$PAHO_ROOT/lib -lpaho-mqttpp3 -lpaho-mqtt3a

    # Helpful for static libs:
    QMAKE_CXXFLAGS += -fPIC

    # Android permissions
    ANDROID_PERMISSIONS += android.permission.WAKE_LOCK
    ANDROID_PERMISSIONS += android.permission.READ_EXTERNAL_STORAGE
    ANDROID_PERMISSIONS += android.permission.WRITE_EXTERNAL_STORAGE
    ANDROID_PERMISSIONS += android.permission.ACCESS_COARSE_LOCATION
    ANDROID_PERMISSIONS += android.permission.ACCESS_FINE_LOCATION
    ANDROID_PERMISSIONS += android.permission.ACCESS_WIFI_STATE
    ANDROID_PERMISSIONS += android.permission.USE_PERIPHERAL_IO
    ANDROID_PERMISSIONS += android.permission.CAMERA
    ANDROID_PERMISSIONS += android.permission.INTENT_ACTION_USB_PERMISSION
    ANDROID_PERMISSIONS += android.permission.INTERNET
    ANDROID_PERMISSIONS += android.permission.ACCESS_NETWORK_STATE
    ANDROID_PERMISSIONS += android.permission.FOREGROUND_SERVICE
    ANDROID_PERMISSIONS += android.hardware.usb.action.USB_DEVICE_ATTACHED
    ANDROID_PERMISSIONS += android.permission.BLUETOOTH
    ANDROID_PERMISSIONS += android.permission.BLUETOOTH_ADMIN
    ANDROID_PERMISSIONS += android.permission.BLUETOOTH_SCAN
    ANDROID_PERMISSIONS += android.permission.BLUETOOTH_ADVERTISE
    ANDROID_PERMISSIONS += android.permission.BLUETOOTH_CONNECT

    # C++ std + linker
    QMAKE_CXXFLAGS += -std=c++17
    QMAKE_LFLAGS += -static-libstdc++ -static-libgcc
    CONFIG += c++17
    CONFIG += mobility
    CONFIG += debug
#    MOBILITY += bluetooth

    # Android extra files
    DISTFILES += \
        android/AndroidManifest.xml \
        android/build.gradle \
        android/gradle.properties \
        android/gradle/wrapper/gradle-wrapper.jar \
        android/gradle/wrapper/gradle-wrapper.properties \
        android/gradlew \
        android/gradlew.bat \
        android/main/java/com/hoho/android/usbserial/driver/DeviceActivity.java \
        android/res/drawable/ic_delete_white_24dp.xml \
        android/res/drawable/ic_send_white_24dp.xml \
        android/res/menu/menu_devices.xml \
        android/res/menu/menu_terminal.xml \
        android/res/mipmap-hdpi/ic_launcher.png \
        android/res/mipmap-mdpi/ic_launcher.png \
        android/res/mipmap-xhdpi/ic_launcher.png \
        android/res/mipmap-xxhdpi/ic_launcher.png \
        android/res/mipmap-xxxhdpi/ic_launcher.png \
        android/res/values/arrays.xml \
        android/res/values/colors.xml \
        android/res/values/strings.xml \
        android/res/values/styles.xml \
        android/res/xml/device_filter.xml \
        android/src/main/java/com/hoho/android/usbserial/driver/CdcAcmSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/Ch34xSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/ChromeCcdSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/CommonUsbSerialPort.java \
        android/src/main/java/com/hoho/android/usbserial/driver/Cp21xxSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/CustomProber.java \
        android/src/main/java/com/hoho/android/usbserial/driver/FtdiSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/GsmModemSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/ProbeTable.java \
        android/src/main/java/com/hoho/android/usbserial/driver/ProlificSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/SerialTimeoutException.java \
        android/src/main/java/com/hoho/android/usbserial/driver/TestClassTerje.java \
        android/src/main/java/com/hoho/android/usbserial/driver/UsbId.java \
        android/src/main/java/com/hoho/android/usbserial/driver/UsbSerialDriver.java \
        android/src/main/java/com/hoho/android/usbserial/driver/UsbSerialPort.java \
        android/src/main/java/com/hoho/android/usbserial/driver/UsbSerialProber.java \
        android/src/main/java/com/hoho/android/usbserial/util/HexDump.java \
        android/src/main/java/com/hoho/android/usbserial/util/MonotonicClock.java \
        android/src/main/java/com/hoho/android/usbserial/util/SerialInputOutputManager.java \
        android/src/main/java/com/hoho/android/usbserial/util/UsbUtils.java \
        android/src/main/java/com/hoho/android/usbserial/util/XonXoffFilter.java \
        android/res/values/libs.xml \
        android/res/xml/qtprovider_paths.xml

    # Your original extra Android includes
    contains(ANDROID_TARGET_ARCH,arm64-v8a) {
        ANDROID_PACKAGE_SOURCE_DIR = $$PWD/../android
    }

    SUBDIRS += \
        android/WitSDK/consumer-rules.pro \
        android/WitSDK/proguard-rules.pro \
        android/imu/proguard-rules.pro
}

DISTFILES += \
    $$PWD/../android/src/main/java/com/hoho/android/usbserial/util/SharedStorage.java

