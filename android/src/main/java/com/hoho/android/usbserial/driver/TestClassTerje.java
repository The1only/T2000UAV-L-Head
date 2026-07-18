// // Java class
// // Debugging... /Users/terjenilsen/AndroidSDK/platform-tools/adb -s 10.19.0.101:5555 logcat | grep TERJE

package com.hoho.android.usbserial.driver;

import android.app.PendingIntent;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.ContextWrapper;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.Spannable;
import android.text.SpannableStringBuilder;
import android.text.method.ScrollingMovementMethod;
import android.text.style.ForegroundColorSpan;
import android.net.Uri;
import android.provider.Settings;

import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;

import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import android.util.Log;

import java.util.ArrayList;
import java.util.Locale;

import androidx.annotation.NonNull;
import androidx.fragment.app.ListFragment;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import com.hoho.android.usbserial.util.SerialInputOutputManager;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.CustomProber;
import com.hoho.android.usbserial.util.HexDump;

import java.net.InetAddress;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

import java.io.IOException;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Objects;
import java.util.ArrayDeque;
import java.util.Queue;

public class TestClassTerje implements SerialInputOutputManager.Listener {

    private String TAG = "USBLOG";
    public long nativeHandle = 0;   // instance field, not static

    // Device id range you care about
    private final int deviceId1 = 1000;
    private final int deviceId2 = 1100;

    private enum UsbPermission { Unknown, Requested, Granted, Denied }
    private final String INTENT_ACTION_GRANT_USB = BuildConfig.APPLICATION_ID + ".GRANT_USB";
    private final int WRITE_WAIT_MILLIS = 200;

    private boolean connected = false;
    private SerialInputOutputManager usbIoManager;
    private int baudRate=9600; // Default for the scanner...
    private int g_portNum = 0;
    private UsbSerialPort usbSerialPort;
    private UsbPermission usbPermission = UsbPermission.Unknown;
    private final boolean withIoManager = true; // event-driven ON
    private final Handler mainLooper;
    private final Context mcontext;

    // USB permission request state
    private int pendingPortNum = -1;
    private int pendingDeviceId = -1;
    private boolean usbPermissionReceiverRegistered = false;
    private boolean usbPermissionStatus = false;

    private static int[] used = new int[5];
    private static String[] _serialnum = { "", "", "", "", "" };
    private static boolean _first = true;

    private String serialNum = "";

    // Optional pull queue (raw chunks as received)
    private final Object qLock = new Object();
    private final Queue<byte[]> rxQueue = new ArrayDeque<>(64);

    // ===== JNI callbacks (implement these in C++) =====
    static {
        try { System.loadLibrary("serialbridge"); } catch (Throwable ignore) {}
    }

    private native void nativeOnSerialBytes(byte[] data);
    private native void nativeOnConnected(boolean connected);
    private native void nativeOnSerialError(String message);

    private static class BuildConfig {
        public static final boolean DEBUG = Boolean.parseBoolean("true");
        public static final String APPLICATION_ID = "com.hoho.android.usbserial.examples";
        public static final String BUILD_TYPE = "debug";
    }

    private void status(String s) { Log.d(TAG, s); }

    public TestClassTerje(Context contextIn) {
        /*
         * Use the application context so this long-lived object does not retain
         * an Activity instance.
         */
        Context applicationContext = contextIn.getApplicationContext();
        this.mcontext =
                applicationContext != null ? applicationContext : contextIn;

        this.mainLooper = new Handler(Looper.getMainLooper());

        registerUsbPermissionReceiver();
    }

    private void registerUsbPermissionReceiver() {
        if (usbPermissionReceiverRegistered) {
            return;
        }

        IntentFilter filter = new IntentFilter(INTENT_ACTION_GRANT_USB);

        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                mcontext.registerReceiver(
                        usbPermissionReceiver,
                        filter,
                        Context.RECEIVER_NOT_EXPORTED
                );
            } else {
                mcontext.registerReceiver(
                        usbPermissionReceiver,
                        filter
                );
            }

            usbPermissionReceiverRegistered = true;
            status("USB permission receiver registered");

        } catch (Exception e) {
            status("Failed to register USB permission receiver: "
                    + e.getMessage());
        }
    }

    private void unregisterUsbPermissionReceiver() {
        if (!usbPermissionReceiverRegistered) {
            return;
        }

        try {
            mcontext.unregisterReceiver(usbPermissionReceiver);
        } catch (IllegalArgumentException ignore) {
            // Receiver was already unregistered.
        } catch (Exception e) {
            status("Failed to unregister USB permission receiver: "
                    + e.getMessage());
        } finally {
            usbPermissionReceiverRegistered = false;
        }
    }

    /**
     * Call this from C++ when the Java serial object is permanently destroyed.
     * Do not call it for an ordinary temporary disconnect.
     */
    public void release() {
        disconnect();
        //unregisterUsbPermissionReceiver();
        nativeHandle = 0;
    }

    private final BroadcastReceiver usbPermissionReceiver =
            new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!INTENT_ACTION_GRANT_USB.equals(intent.getAction())) {
                return;
            }

            UsbDevice device;

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                device = intent.getParcelableExtra(
                        UsbManager.EXTRA_DEVICE,
                        UsbDevice.class
                );
            } else {
                device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            }

            boolean permissionGranted = intent.getBooleanExtra(
                    UsbManager.EXTRA_PERMISSION_GRANTED,
                    false
            );

            if (device == null) {
                status("USB permission result did not contain a device");
                usbPermission = UsbPermission.Unknown;
                pendingPortNum = -1;
                pendingDeviceId = -1;
                return;
            }

            // Ignore a permission result belonging to an older request.
            if (pendingDeviceId != -1 &&
                    device.getDeviceId() != pendingDeviceId) {
                status("Ignoring USB permission result for another device");
                return;
            }

            int portToOpen = pendingPortNum;

            pendingPortNum = -1;
            pendingDeviceId = -1;

            if (!permissionGranted) {
                usbPermission = UsbPermission.Denied;
                status("USB permission denied");
                try {
                    nativeOnSerialError("USB permission denied");
                } catch (Throwable ignore) {
                }
                return;
            }

            usbPermission = UsbPermission.Granted;
            status("USB permission granted");
            usbPermissionStatus = true;

            /*
             * requestPermission() is asynchronous. Now that permission has
             * actually been granted, call connect() again to open the device.
             */
            if (portToOpen >= 0) {
                connect(portToOpen);
            }
        }
    };

    // ---------------------- Public API (called from C++) ----------------------

    /** Returns last known device serial (string meta info; not used for I/O). */
    public String getInfo() { return this.serialNum; }

    /** Disconnect and return a tiny status (purely informational). */
    public String disconn() { disconnect(); return "Discon"; }

    /** Send raw bytes exactly as provided. */
    public int sendToSerial(byte[] bytes) { return send(bytes); }

    /** Pull next received chunk (or null if none). */
    public byte[] recFromSerial() {
        synchronized (qLock) { return rxQueue.poll(); }
    }

    public String usbpremission() {
        Boolean x = usbPermissionStatus;
        status("Debug USB: " + x);
        String xval = Boolean.toString(x);
        return String.valueOf(xval);
    }

    /** Connect by numeric port index (0..n). */
    public String getconnected(int port, int baudrate) {
        this.TAG = "USBLOG_" + port;
        this.baudRate = baudrate;
        connect(port);
        return String.valueOf(this.connected);
    }

    /** Connect by matching device serial string + set baudrate. */
    public String connectserial(String serial, int baudrate) {
        this.TAG = "USBLOG_" + serial;
        this.baudRate = baudrate;
        status("Debug 1: " + serial);
//        connects(serial);
        int stat = connect(Integer.parseInt(serial));
        String xval = Integer.toString(stat);
        return String.valueOf(xval);
    }

    // ---------------------- SerialInputOutputManager.Listener -----------------
    public void setNativeHandle(long ptr) {
        nativeHandle = ptr;

        if(_first == true){
            buildSerialNumbers();
            _first = false;
        }

        android.util.Log.d("USBLOG", "setNativeHandle=" + ptr);
    }

    /** Event-driven receive: push raw bytes to C++ and queue for optional pull. */
    @Override
    public void onNewData(byte[] data) {
        try { nativeOnSerialBytes(data); } catch (Throwable ignore) {}
    }

    @Override
    public void onRunError(Exception e) {
        mainLooper.post(() -> {
            status("IO loop ended: " + e.getMessage());
            try { nativeOnSerialError(e.getMessage()); } catch (Throwable ignore) {}
            disconnect();
        });
    }

    // ---------------------- Connection management -----------------------------

    private static String norm(String s) {
        if (s == null) return null;
        s = s.trim();
        return s.replace("\u200B", "").replace("\uFEFF", "");
    }

    private void buildSerialNumbers() {
        status("TERJE::: build serial");

        for (int i = 0; i < _serialnum.length; i++) {
            connect(i);

            if (connected) {
                _serialnum[i] = serialNum;

                status(String.format(
                        Locale.US,
                        "Device serial: '%s'",
                        _serialnum[i]
                ));

                disconnect();
            }
        }
    }

    private void connects(String targetSerial) {
        for (int i = 0; i < _serialnum.length; i++) {
            String a = norm(targetSerial);
            String b = norm(_serialnum[i]);

            status(String.format(
                    Locale.US,
                    "Compare serial: '%s' vs '%s'",
                    a,
                    b
            ));

            if (Objects.equals(a, b)) {
                connect(i);

                if (connected) {
                    return;
                }
            }
        }
    }

    private void findall() {
        for (int i = 0; i < _serialnum.length; i++) {
            String b = norm(_serialnum[i]);

            status(String.format(
                    Locale.US,
                    "Found serial: '%s'",
                    b
            ));
        }
    }

// ---------------------- Connection management -----------------------------

    private int connect(int portNum) {
        final int requestedPortNum = portNum;

        if (connected) {
            status("already connected");
            return 0;
        }

        UsbManager usbManager =
                (UsbManager) mcontext.getSystemService(Context.USB_SERVICE);

        if (usbManager == null) {
            status("connection failed: UsbManager unavailable");
            return -1;
        }

        UsbDevice device = null;

        for (UsbDevice candidate : usbManager.getDeviceList().values()) {
            if (candidate.getDeviceId() > deviceId1 &&
                    candidate.getDeviceId() < deviceId2) {

                if (portNum == 0) {
                    device = candidate;
                    break;
                }

                portNum--;
            }
        }

        if (device == null) {
            status("connection failed: device not found");
            return -2;
        }

        UsbSerialDriver driver =
                UsbSerialProber.getDefaultProber().probeDevice(device);

        if (driver == null) {
            driver = CustomProber.getCustomProber().probeDevice(device);
        }

        if (driver == null) {
            status("connection failed: no driver for device");
            return -3;
        }

        if (driver.getPorts().isEmpty()) {
            status("connection failed: driver contains no ports");
            return -4;
        }

        /*
         * Do not call openDevice() before permission exists.
         *
         * requestPermission() returns immediately. Android later delivers the
         * result to usbPermissionReceiver.
         */
        if (!usbManager.hasPermission(device)) {
            if (usbPermission != UsbPermission.Requested ||
                    pendingDeviceId != device.getDeviceId()) {

                usbPermission = UsbPermission.Requested;
                pendingPortNum = requestedPortNum;
                pendingDeviceId = device.getDeviceId();

                Intent permissionResultIntent =
                        new Intent(INTENT_ACTION_GRANT_USB);

                permissionResultIntent.setPackage(
                        mcontext.getPackageName()
                );

                int flags = PendingIntent.FLAG_UPDATE_CURRENT;

                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                    flags |= PendingIntent.FLAG_MUTABLE;
                }

                PendingIntent usbPermissionIntent =
                        PendingIntent.getBroadcast(
                                mcontext,
                                device.getDeviceId(),
                                permissionResultIntent,
                                flags
                        );

                status("requesting USB permission for port "
                        + requestedPortNum);

                usbManager.requestPermission(
                        device,
                        usbPermissionIntent
                );
            } else {
                status("USB permission request already pending");
//                return -6;
            }

            return -5;
        }

        usbPermission = UsbPermission.Granted;
        pendingPortNum = -1;
        pendingDeviceId = -1;

        UsbDeviceConnection usbConnection =
                usbManager.openDevice(device);

        if (usbConnection == null) {
            status(
                    !usbManager.hasPermission(device)
                            ? "permission denied"
                            : "openDevice failed"
            );
            return -7;
        }

        usbSerialPort = driver.getPorts().get(0);

        try {
            usbSerialPort.open(usbConnection);
            getInfo(driver);

            try {
                usbSerialPort.setParameters(
                        baudRate,
                        8,
                        UsbSerialPort.STOPBITS_1,
                        UsbSerialPort.PARITY_NONE
                );
            } catch (UnsupportedOperationException e) {
                status("setParameters unsupported: "
                        + e.getMessage());
            }

            if (withIoManager) {
                usbIoManager =
                        new SerialInputOutputManager(
                                usbSerialPort,
                                this
                        );

                usbIoManager.start();
            }

            connected = true;

            /*
             * Store the original requested port number. The local portNum
             * variable was decremented while searching for the device.
             */
            g_portNum = requestedPortNum;

            usbSerialPort.setDTR(true);

            status("connected to port " + requestedPortNum);

            try {
                nativeOnConnected(true);
            } catch (Throwable ignore) {
            }

        } catch (Exception e) {
            status("connection failed: " + e.getMessage());

            /*
             * If UsbSerialPort.open() failed, its connection might not have
             * been adopted by the port implementation.
             */
            if (usbSerialPort == null) {
                try {
                    usbConnection.close();
                } catch (Exception ignore) {
                }
            }

            disconnect();
        }
        return 1;
    }


    private void getInfo(UsbSerialDriver driver) {
        UsbDevice usbDev = driver.getDevice();
        try {
            serialNum = usbDev.getSerialNumber();
            status("Serial=" + serialNum);
        } catch (Exception e) {
            serialNum = "";
            status("Serial not available: " + e.getMessage());
        }
    }

    private void disconnect() {
        boolean wasConnected = connected;

        connected = false;

        if (usbIoManager != null) {
            try {
                usbIoManager.setListener(null);
                usbIoManager.stop();
            } catch (Exception ignore) {
            }

            usbIoManager = null;
        }

        if (usbSerialPort != null) {
            try {
                usbSerialPort.close();
            } catch (IOException ignore) {
            } catch (Exception ignore) {
            }

            usbSerialPort = null;
        }

        /*
         * This only resets the Java request state. Android's granted USB
         * permission remains valid until the device is detached.
         */
        if (usbPermission != UsbPermission.Requested) {
            usbPermission = UsbPermission.Unknown;
            pendingPortNum = -1;
            pendingDeviceId = -1;
        }

        if (wasConnected) {
            try {
                nativeOnConnected(false);
            } catch (Throwable ignore) {
            }
        }

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // ---------------------- Raw send (bytes) ---------------------------------

    private int send(byte[] data) {
        if (!connected) {
            status("TERJE::: not connected");
            try {
                disconnect();
                Thread.sleep(100);
                connect(g_portNum);
                Thread.sleep(100);
            } catch (Exception e) {
                status("TERJE::: send reconnect failed: " + e.getMessage());
                return -1;
            }
        }
        try {
          //  status("Sending: " + data.toString());
            usbSerialPort.write(data, WRITE_WAIT_MILLIS);
            return data.length;
        } catch (Exception e) {
            onRunError(e);
            return -1;
        }
    }

    // ---------------------- Control lines ------------------------------------

    public int ControlLines(boolean rts /*cts ignored; it's an input*/) {
        if (!connected) return -1;
        try {
            usbSerialPort.setRTS(rts);
            usbSerialPort.setDTR(true);
            EnumSet<UsbSerialPort.ControlLine> lines = usbSerialPort.getControlLines();
            // Example: boolean cts = lines.contains(UsbSerialPort.ControlLine.CTS);
        } catch (Exception e) {
            status("control lines failed: " + e.getMessage());
        }
        return (rts ? 1 : 0);
    }

// ---------------------- Control lines ------------------------------------

    public int change(int n)
    {
        int r = setScreenBrightness( mcontext, n);
        return r;
    }

    //  finally use below method for set brightness
    private int setScreenBrightness(Context xContext,int brightnessValue){
        // Make sure brightness value between 0 to 255

        if (!Settings.System.canWrite(xContext))
        {
            showBrightnessPermissionDialog(xContext);
            return -1;
        }

        if(brightnessValue >= 0 && brightnessValue <= 255){
            Settings.System.putInt(
                xContext.getContentResolver(),
                Settings.System.SCREEN_BRIGHTNESS,
                brightnessValue
            );
        }
        return brightnessValue;
    }

    private static void showBrightnessPermissionDialog(final Context context) {

        Intent intent = new Intent(android.provider.Settings.ACTION_MANAGE_WRITE_SETTINGS);
        intent.setData(Uri.parse("package:" + context.getPackageName()));
        intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        context.startActivity(intent);
    }

}
