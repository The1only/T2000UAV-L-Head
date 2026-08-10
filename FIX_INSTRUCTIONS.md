# Android getentropy Fix Instructions
# Fixes: java.lang.UnsatisfiedLinkError: cannot locate symbol "getentropy"

## Problem
The Qt6Core library was built for Android API 21, but getentropy() requires API 28+.
Your app manifest specifies minSdkVersion="28" but native libraries are built for API 21.

## Solution: Rebuild GeographicLib for API 28+

### Step 1: Run the rebuild script
```bash
chmod +x rebuild_geographiclib_android.sh
./rebuild_geographiclib_android.sh
```

### Step 2: Clean and rebuild your Qt project
```bash
# In Qt Creator or command line:
qmake clean
qmake
make
```

### Step 3: Check file paths
Verify these files exist after rebuild:
- `GeographicLib/build-arm64-api28/install/lib/libGeographicLib.so`
- `GeographicLib/build-arm64-api28/install/include/GeographicLib/`

### Step 4: Test the app
Deploy and run the Android app - the getentropy error should be resolved.

## Files Modified
- `platform/android.pri` - Updated library paths to API 28 build
- Android native library build target changed from API 21 → API 28

## Alternative Quick Fix
If rebuilding libraries is not possible, you can lower the Android API:
In `android/AndroidManifest.xml`, change:
```xml
<uses-sdk android:minSdkVersion="21" android:targetSdkVersion="28"/>
```
But this limits your app to older Android versions.

## Verification
After fixing, the error should no longer occur:
- No more "dlopen failed: cannot locate symbol getentropy" 
- Qt6Core library loads successfully
- App starts normally on Android devices