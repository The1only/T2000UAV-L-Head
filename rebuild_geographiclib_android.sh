#!/bin/bash
# Script to rebuild GeographicLib for Android API 28+
# This fixes the getentropy symbol missing error

cd GeographicLib
mkdir -p build-arm64-api28
cd build-arm64-api28

export NDK=~/Library/Android/sdk/ndk/27.2.12479018

cmake ../ \
    -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
    -DANDROID_ABI=arm64-v8a \
    -DANDROID_PLATFORM=android-28 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$(pwd)/install

cmake --build . --target install -- -j$(nproc)

echo "GeographicLib rebuilt for Android API 28"
echo "Update android.pri to use: $$PWD/../GeographicLib/build-arm64-api28/install/lib"