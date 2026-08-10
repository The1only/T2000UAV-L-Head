# Alternative Fix: Lower Android API Target
# If you can't rebuild GeographicLib, you can lower your Android target

# In android/AndroidManifest.xml, change:
# FROM: <uses-sdk android:minSdkVersion="28" android:targetSdkVersion="31"/>
# TO:   <uses-sdk android:minSdkVersion="21" android:targetSdkVersion="28"/>

# This matches your existing GeographicLib build (API 21)
# But removes access to newer Android features (API 28+)

# WARNING: This approach may cause other compatibility issues
# and limits your app to older Android versions.
# Rebuilding for API 28+ is the recommended approach.