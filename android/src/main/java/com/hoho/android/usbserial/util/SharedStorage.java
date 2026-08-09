package com.hoho.android.usbserial.util;

import android.content.ContentResolver;
import android.content.ContentUris;
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.net.Uri;
import android.os.Build;
import android.provider.MediaStore;
import android.util.Log;

import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

public class SharedStorage
{
    private static final String TAG = "SharedStorage";

    /*
     * Append text to:
     *
     * Download/<subDirectory>/<fileName>
     *
     * For example:
     *
     * Download/LowEnergyScanner/transponder_log.txt
     */
    public static boolean appendTextFile(
            Context context,
            String subDirectory,
            String fileName,
            String text)
    {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            Log.e(TAG, "Android versions below Android 10 are not supported");
            return false;
        }

        if (context == null) {
            Log.e(TAG, "Android context is null");
            return false;
        }

        try {
            ContentResolver resolver =
                    context.getContentResolver();

            String relativePath =
                    "Download/" + subDirectory + "/";

            Uri collection =
                    MediaStore.Downloads.getContentUri(
                            MediaStore.VOLUME_EXTERNAL_PRIMARY);

            Uri fileUri = findFile(
                    resolver,
                    collection,
                    fileName,
                    relativePath);

            if (fileUri == null) {
                fileUri = createTextFile(
                        resolver,
                        collection,
                        fileName,
                        relativePath);

                if (fileUri == null) {
                    return false;
                }
            }

            /*
             * "wa" means write and append.
             */
            try (OutputStream output =
                         resolver.openOutputStream(fileUri, "wa")) {

                if (output == null) {
                    Log.e(TAG, "openOutputStream returned null");
                    return false;
                }

                output.write(
                        text.getBytes(StandardCharsets.UTF_8));

                output.flush();
            }

            return true;
        }
        catch (Exception exception) {
            Log.e(
                    TAG,
                    "Could not append text file",
                    exception);

            return false;
        }
    }

    /*
     * Replace the complete contents of an existing file,
     * or create it if it does not exist.
     */
    public static boolean writeTextFile(
            Context context,
            String subDirectory,
            String fileName,
            String text)
    {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            Log.e(TAG, "Android versions below Android 10 are not supported");
            return false;
        }

        if (context == null) {
            Log.e(TAG, "Android context is null");
            return false;
        }

        try {
            ContentResolver resolver =
                    context.getContentResolver();

            String relativePath =
                    "Download/" + subDirectory + "/";

            Uri collection =
                    MediaStore.Downloads.getContentUri(
                            MediaStore.VOLUME_EXTERNAL_PRIMARY);

            Uri fileUri = findFile(
                    resolver,
                    collection,
                    fileName,
                    relativePath);

            if (fileUri == null) {
                fileUri = createTextFile(
                        resolver,
                        collection,
                        fileName,
                        relativePath);

                if (fileUri == null) {
                    return false;
                }
            }

            /*
             * "w" means write and overwrite.
             */
            try (OutputStream output =
                         resolver.openOutputStream(fileUri, "w")) {

                if (output == null) {
                    Log.e(TAG, "openOutputStream returned null");
                    return false;
                }

                output.write(
                        text.getBytes(StandardCharsets.UTF_8));

                output.flush();
            }

            return true;
        }
        catch (Exception exception) {
            Log.e(
                    TAG,
                    "Could not write text file",
                    exception);

            return false;
        }
    }

    /*
     * Create a new text file in MediaStore.
     */
    private static Uri createTextFile(
            ContentResolver resolver,
            Uri collection,
            String fileName,
            String relativePath)
    {
        try {
            ContentValues values =
                    new ContentValues();

            values.put(
                    MediaStore.MediaColumns.DISPLAY_NAME,
                    fileName);

            values.put(
                    MediaStore.MediaColumns.MIME_TYPE,
                    "text/plain");

            values.put(
                    MediaStore.MediaColumns.RELATIVE_PATH,
                    relativePath);

            Uri fileUri =
                    resolver.insert(collection, values);

            if (fileUri == null) {
                Log.e(TAG, "MediaStore insert returned null");
            }

            return fileUri;
        }
        catch (Exception exception) {
            Log.e(
                    TAG,
                    "Could not create text file",
                    exception);

            return null;
        }
    }

    /*
     * Search for an existing file with the same filename
     * and relative path.
     */
    private static Uri findFile(
            ContentResolver resolver,
            Uri collection,
            String fileName,
            String relativePath)
    {
        String[] projection = {
                MediaStore.MediaColumns._ID
        };

        String selection =
                MediaStore.MediaColumns.DISPLAY_NAME + " = ? AND " +
                MediaStore.MediaColumns.RELATIVE_PATH + " = ?";

        String[] selectionArguments = {
                fileName,
                relativePath
        };

        try (Cursor cursor = resolver.query(
                collection,
                projection,
                selection,
                selectionArguments,
                null)) {

            if (cursor != null && cursor.moveToFirst()) {
                int idColumn =
                        cursor.getColumnIndexOrThrow(
                                MediaStore.MediaColumns._ID);

                long id = cursor.getLong(idColumn);

                return ContentUris.withAppendedId(
                        collection,
                        id);
            }
        }
        catch (Exception exception) {
            Log.e(
                    TAG,
                    "Could not search for existing file",
                    exception);
        }

        return null;
    }
}
