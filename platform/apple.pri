# platform/apple.pri
# macOS and iOS settings

# ----------------
# macOS
# ----------------
macx {
    message(This is MAC....)

    CONFIG += app_bundle
    QMAKE_INFO_PLIST = $$PWD/Info.plist

    QT += serialport multimedia bluetooth

    # C++ standard
    CONFIG += c++17
    QMAKE_CXXFLAGS += -std=c++17

    message("Using paho.mqtt.cpp and paho.mqtt.c")
    INCLUDEPATH += /usr/local/include
    LIBS += -L/usr/local/lib

    # You had both of these; keep just one set
    LIBS += -lpaho-mqttpp3 -lpaho-mqtt3a
    LIBS -= framework AGL

    # prefer opt/homebrew on Apple Silicon
    exists($$PWD/../GeographicLib) {
        message(... FOUND ... )
        INCLUDEPATH += $$PWD/../GeographicLib/build-mac/install/include
        LIBS += -L$$PWD/../GeographicLib/build-mac/install/lib -lGeographicLib
        GEODATA_DIR = //usr/local/share/GeographicLib
#       INCLUDEPATH += /opt/homebrew/include
#       LIBS += -L/opt/homebrew/lib -lGeographicLib
#       GEODATA_DIR = /opt/homebrew/share/GeographicLib
    } else {
        message(... ERROR ... Missing: $$PWD/../GeographicLib/build-mac/install/include)
    }

    DEFINES += USE_GEOGRAPHICLIB

    SOURCES += \
        ./Source/generic/main.cpp \
        ./Source/generic/mainwindow.cpp \
        ./Source/generic/mytcpsocket.cpp \
        ./Source/driver/serialport.cpp \

#        ./Source/generic/multicastlistner.cpp \


    HEADERS += \
        ./Header/generic/mainwindow.h \
        ./Header/generic/mytcpsocket.h \
        ./Header/generic/reg.h \
        ./Header/driver/serialport.h \

}

# ----------------
# iOS
# ----------------
# HUSK: User Script Sandboxing = NO must be set in XCODE...
ios {
    message(This is iPhone....)

    CONFIG += add_ios_ffmpeg_libraries
    CONFIG += sdk_no_version_check
    QT +=  multimedia

    QTPLUGIN -= qffmpeg

    # 👇 turn off Xcode's user script sandboxing for this target
    QMAKE_XCODE_ATTRIBUTE_ENABLE_USER_SCRIPT_SANDBOXING = NO

    # iOS bundle / signing
    QMAKE_TARGET_BUNDLE_IDENTIFIER = com.example.lowenergyscanner
    QMAKE_MAC_SDK = iphoneos
    QMAKE_XCODE_CODE_SIGN_STYLE = Automatic
    QMAKE_XCODE_DEVELOPMENT_TEAM =
    QMAKE_XCODE_CODE_SIGN_IDENTITY = ""
    QMAKE_XCODE_PROVISIONING_PROFILE = ""
    QMAKE_BUNDLE_IDENTIFIER =

# your iOS paho path
    PAHO_IOS_BASE = /Users/terjenilsen/Dropbox/Sportsfly/transponder/third_party/paho/ios
    PAHO_IOS = $$PAHO_IOS_BASE/iphoneos
    INCLUDEPATH += $$PAHO_IOS/include
    INCLUDEPATH += $$PWD/../Header/generic
    INCLUDEPATH += $$PWD/../Header/driver
    INCLUDEPATH += $$PWD/../example/Headers
    INCLUDEPATH += ./Header/generic
    INCLUDEPATH += ./Header/driver
    INCLUDEPATH += ./example/Headers
    LIBS += -L$$PAHO_IOS/lib -lpaho-mqttpp3 -lpaho-mqtt3a

    #-------------------------------------------
    # prefer opt/homebrew on Apple Silicon
    exists($$PWD/../GeographicLib) {
        message(... FOUND ... )
        INCLUDEPATH += $$PWD/../GeographicLib/build-ios/include
        INCLUDEPATH += $$PWD/../GeographicLib/include

        # If the lib is libGeographicLib.a:
        LIBS += $$PWD/../GeographicLib/build-ios/src/Release-iphoneos/libGeographicLib.a

        # If instead it is libGeographicLib_STATIC.a, use this:
        # LIBS += $$GEOLIB_ROOT/build-ios/build/Release-iphoneos/libGeographicLib_STATIC.a
    } else {
        error(... ERROR ... Missing: $$PWD/../GeographicLib/build-ios/install/include)
    }

    DEFINES += USE_GEOGRAPHICLIB
    #-------------------------------------------

    # iOS sources
    SOURCES += \
        $$PWD/../Source/generic/main.cpp \
        $$PWD/../Source/generic/mainwindow.cpp \
        $$PWD/../Source/generic/mytcpsocket.cpp \

#        $$PWD/../Source/generic/multicastlistner.cpp \

#$$PWD/../Source/generic/myNativeWrapperFunctions.mm \


    HEADERS += \
        ./Header/generic/mainwindow.h \
        ./Header/generic/mytcpsocket.h \
        ./Header/generic/REG.h \

#./Header/generic/multicastlistner.h \

# WidgetAI.h \

#./Header/generic/myNativeWrapperFunctions.h \

    # Position plist for iOS
    QMAKE_INFO_PLIST = $$PWD/Info.plist

    # Recommended for static 3rd party on iOS
    QMAKE_CXXFLAGS += -fPIC
}

# Optional: if you still want these convenience lines:
ios: QMAKE_INFO_PLIST = Info.plist
macos: QMAKE_INFO_PLIST = Info.qmake.macos.plist

# ----------------------------------
DOXYGEN = doxygen
DOXYFILE = $$PWD/docs/Doxyfile

doxygen.target = doc
doxygen.commands = $$DOXYGEN $$DOXYFILE
doxygen.CONFIG += no_link

QMAKE_EXTRA_TARGETS += doxygen

