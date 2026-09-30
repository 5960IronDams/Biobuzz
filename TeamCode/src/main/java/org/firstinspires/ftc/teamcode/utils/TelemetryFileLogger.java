package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Records Panels telemetry + explicit events to a file on the RC for debugging.
 *
 * <p><b>Usage:</b>
 * <pre>
 * // In RobotMain constructor (appContext comes from hardwareMap):
 * logger = TelemetryFileLogger.create(opmode.hardwareMap.appContext);
 *
 * // In flushTelemetry():
 * logger.captureSnapshot(DashTelemetry);
 * DashTelemetry.update();
 *
 * // When done (OpMode.stop()):
 * logger.close();
 * </pre>
 *
 * <p>Logs are written to the app-private external files dir when available
 * ({@code /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/telemetry/...},
 * pullable via ADB), falling back to internal storage
 * ({@code /data/user/0/.../files/telemetry/...}).
 *
 * <p>Why not {@code /data/local/tmp}? That path is owned by shell/root. The RC
 * app runs as its own UID and gets {@code Permission denied} there (SELinux +
 * Unix perms). App-private dirs are always writable with no extra permission.
 *
 * <p><b>File naming / when is a log safe to pull:</b> while writing, the file
 * is {@code ftc_telemetry_<timestamp>.log.open}; {@link #close()} renames it
 * to {@code ftc_telemetry_<timestamp>.log}. Only {@code .log} files are
 * complete (they end with the "Log Ended" marker) — {@code .open} files are
 * mid-write or were abandoned (app killed before {@code stop()}).
 *
 * <p>Why the rename dance: the Hub runs Android 7, whose FUSE layer serves
 * STALE file sizes to other processes (adb/MTP) for freshly written or
 * recently closed files — pulling seconds after {@code close()} truncates the
 * copy at an old size (verified via logcat 2026-09-29: file closed 22:19:39,
 * copy cut at its size from ~22:19:19). The rename creates a fresh directory
 * entry with fresh attributes, so pulls of {@code .log} files are complete.
 * Even so, give a just-closed file a few seconds before pulling.
 */
public class TelemetryFileLogger {
    private static final String TAG = "FtcTelemetryLogger";
    private static final String LOG_SUBDIR = "telemetry";

    private BufferedWriter writer;
    private File logFile;
    private long startTimeMs;
    private boolean enabled;

    /**
     * Creates a new logger and opens the log file.
     * Returns null if file I/O fails (callers must null-check).
     */
    public static TelemetryFileLogger create(Context context) {
        return create(context, null);
    }

    /**
     * Creates a new logger, tagging the header with the owning OpMode name.
     * Each {@code RobotMain} construction (every init) opens a NEW timestamped
     * file — so switching OpModes or re-pressing init abandons the previous
     * file unclosed (no END marker). The tag tells you which file belongs to
     * which run when several appear seconds apart.
     */
    public static TelemetryFileLogger create(Context context, String tag) {
        TelemetryFileLogger logger = new TelemetryFileLogger();
        if (logger.init(context, tag)) {
            return logger;
        }
        return null;
    }

    private TelemetryFileLogger() {
        this.enabled = true;
        this.startTimeMs = System.currentTimeMillis();
    }

    private boolean init(Context context, String tag) {
        try {
            File logDir = resolveLogDir(context);
            if (!logDir.exists() && !logDir.mkdirs()) {
                Log.w(TAG, "Failed to create log directory: " + logDir.getAbsolutePath());
                return false;
            }
            if (!logDir.canWrite()) {
                Log.w(TAG, "Log directory not writable: " + logDir.getAbsolutePath());
                return false;
            }

            // Timestamped file, seconds resolution. Two inits in the same second
            // would collide, so disambiguate with a counter suffix.
            String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
                    .format(new Date());
            logFile = uniqueFile(logDir, "ftc_telemetry_" + timestamp);

            writer = new BufferedWriter(new FileWriter(logFile, true));
            writer.write("===== FTC Telemetry Log Started: " + new Date()
                    + (tag != null ? " [" + tag + "]" : "") + " =====\n");
            writer.flush();

            Log.i(TAG, "Telemetry logger created: " + logFile.getAbsolutePath());
            return true;
        } catch (IOException | SecurityException e) {
            Log.e(TAG, "Failed to initialize telemetry logger", e);
            enabled = false;
            return false;
        }
    }

    /**
     * In-progress files carry a {@code .log.open} suffix; {@link #close()}
     * renames to {@code .log} when complete. Same-second inits (re-init,
     * OpMode switch) get _2, _3, ... suffixes. The existence check covers
     * BOTH suffixes so a leftover orphan never collides with a final name.
     */
    private static File uniqueFile(File dir, String base) {
        int n = 1;
        while (true) {
            String stem = n == 1 ? base : base + "_" + n;
            File open = new File(dir, stem + ".log.open");
            File done = new File(dir, stem + ".log");
            if (!open.exists() && !done.exists()) return open;
            n++;
        }
    }

    /**
     * Writable-dir resolution order:
     * 1. App-private external files ({@code getExternalFilesDir}) — survives
     *    uninstall? No, but pullable over USB/ADB and large enough.
     * 2. App-private internal files ({@code getFilesDir}) — always writable.
     * 3. Legacy {@code /sdcard/FIRST/telemetry} — only if external storage is mounted.
     */
    private static File resolveLogDir(Context context) {
        if (context != null) {
            try {
                File ext = context.getExternalFilesDir(null);
                if (ext != null) {
                    return new File(ext, LOG_SUBDIR);
                }
            } catch (Exception e) {
                Log.w(TAG, "getExternalFilesDir failed, trying internal", e);
            }
            try {
                return new File(context.getFilesDir(), LOG_SUBDIR);
            } catch (Exception e) {
                Log.w(TAG, "getFilesDir failed, trying /sdcard/FIRST", e);
            }
        }
        File sdcard = Environment.getExternalStorageDirectory();
        return new File(sdcard, "FIRST/" + LOG_SUBDIR);
    }

    /**
     * Captures current telemetry snapshot with timestamp.
     * Call this BEFORE telemetry.update() in your flush cycle.
     *
     * <p>Prefers the structured {@link TelemetrySnapshot} extractor; falls back
     * to {@code toString()} when reflection finds nothing.
     */
    public void captureSnapshot(Telemetry telemetry) {
        if (!enabled || writer == null || telemetry == null) return;

        try {
            long elapsedMs = System.currentTimeMillis() - startTimeMs;
            String timestamp = String.format(Locale.US, "[%6d ms]", elapsedMs);

            String telemetryStr = TelemetrySnapshot.captureDetail(telemetry);

            writer.write(timestamp + " " + telemetryStr + "\n");

            // Flush every snapshot for real-time visibility (can reduce for performance)
            writer.flush();
        } catch (IOException e) {
            Log.e(TAG, "Failed to write telemetry snapshot", e);
            enabled = false;
        }
    }

    /**
     * Capture a single key-value pair directly.
     * Useful for fine-grained control over what gets logged.
     */
    public void log(String key, Object value) {
        if (!enabled || writer == null) return;

        try {
            long elapsedMs = System.currentTimeMillis() - startTimeMs;
            String timestamp = String.format(Locale.US, "[%6d ms]", elapsedMs);
            writer.write(timestamp + " " + key + " = " + value + "\n");
            writer.flush();
        } catch (IOException e) {
            Log.e(TAG, "Failed to write telemetry line", e);
            enabled = false;
        }
    }

    /**
     * Closes the log file and finalizes recording. Safe to call twice.
     *
     * <p>Writes the END marker, closes the writer, then renames
     * {@code <name>.log.open} → {@code <name>.log} so pullers (adb/MTP) get a
     * fresh FUSE directory entry instead of a stale cached size — see the
     * class javadoc. If the app dies before this runs, the file stays
     * {@code .open}: treat any {@code .open} file as incomplete.
     */
    public void close() {
        if (writer != null) {
            try {
                writer.write("===== FTC Telemetry Log Ended: " + new Date() + " =====\n");
                writer.close();
                // Fresh-dentry rename so adb/MTP pulls are never size-truncated.
                File finalized = new File(logFile.getParentFile(),
                        logFile.getName().replaceFirst("\\.open$", ""));
                if (logFile.renameTo(finalized)) {
                    logFile = finalized;
                } else {
                    Log.w(TAG, "Rename to final .log name failed; file remains: "
                            + logFile.getAbsolutePath());
                }
                Log.i(TAG, "Telemetry logger closed. File: " + logFile.getAbsolutePath()
                        + " — pull with: adb pull \"" + logFile.getAbsolutePath() + "\"");
            } catch (IOException e) {
                Log.e(TAG, "Failed to close telemetry logger", e);
            } finally {
                writer = null;
            }
        }
    }

    /**
     * Returns the path to the log file.
     */
    public String getLogFilePath() {
        return logFile != null ? logFile.getAbsolutePath() : "Not created";
    }

    /**
     * Disables logging without closing the file (temporary pause).
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled && writer != null;
    }
}
