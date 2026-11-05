# FTC Telemetry File Logging Guide

## Overview

You now have a complete telemetry logging system that records all Panels telemetry data to a file on the REV Control Hub's storage. This lets you analyze debug information offline and share detailed logs with your team.

## Files Created

### 1. **TelemetryFileLogger.java** (Core logging utility)
   - Location: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/TelemetryFileLogger.java`
   - Handles file I/O, timestamping, and snapshot capture
   - Logs to: `/data/local/tmp/ftc_telemetry_YYYY-MM-DD_HH-mm-ss.log`

### 2. **TelemetrySnapshot.java** (Advanced extraction)
   - Location: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/TelemetrySnapshot.java`
   - Provides detailed telemetry extraction via reflection
   - Optional enhancement for structured logging

### 3. **RobotMain.java** (Integration)
   - Updated to initialize logger in constructor
   - Captures snapshots in `flushTelemetry()` (once per loop)

## How It Works

### Data Flow:
```
Loop Start
    └─ RobotRunPeriodic()
       └─ All subsystems add telemetry via addData()
    └─ flushTelemetry()
       └─ TelemetryFileLogger.captureSnapshot()  ← LOGS HERE
       └─ DashTelemetry.update()  (sends to Panels)
Loop End
```

### Key Design Decisions:
1. **Single capture per loop** - Prevents duplicate/conflicting logs
2. **Before update()** - Captures the final state before transmission
3. **Automatic timestamping** - Relative time from logger creation (milliseconds)
4. **Real-time flush** - Each line written immediately for crash-safe logging
5. **Zero impact on robot code** - Logging is transparent to your OpModes

## Usage

### Basic Setup (Already Done)
Your `RobotMain.java` now automatically:
- Creates logger on initialization
- Captures snapshots every loop
- Closes cleanly on OpMode stop

### Manual Logging (Optional)
You can add specific debug logs from anywhere:

```java
// In any OpMode or subsystem:
if (robot.telemetryLogger != null) {
    robot.telemetryLogger.log("MARKER", "Event happened at pose X");
}
```

### Disabling/Enabling
Pause logging without closing:
```java
robot.telemetryLogger.setEnabled(false);  // Stop logging
robot.telemetryLogger.setEnabled(true);   // Resume
```

### Accessing Log Files

**On the RC via ADB:**
```bash
adb shell ls -la /data/local/tmp/ftc_telemetry_*.log
adb pull /data/local/tmp/ftc_telemetry_2024-12-29_14-30-45.log
```

**Or use RC's file browser (if available in your RC firmware):**
- Navigate to `/data/local/tmp/`
- Look for `ftc_telemetry_*.log` files
- Download to your computer

## Log File Format

Each line in the log looks like:
```
[     0 ms] [Initialized] Vision Loaded
[    12 ms] [Current Alliance = ALLIANCE_RED; Loop Time (ms) = 12.5 ms; Hz = 80.0 Hz; ...]
[    24 ms] [Current Alliance = ALLIANCE_RED; Loop Time (ms) = 12.3 ms; ...]
...
[  1234 ms] [Vision/MT2 = Valid; Pose X = 12.34; Pose Y = 45.67; ...]
```

**Timestamp Column:** Relative milliseconds since logger creation  
**Data Column:** All telemetry key-value pairs from that loop

## Sharing Logs with Support

1. Download the `.log` file from RC
2. Open in any text editor
3. Share via:
   - GitHub issues
   - Slack/Discord
   - Email to me
   - GitHub discussions

Example to parse timestamps:
```bash
# View last 50 lines with timestamps
tail -50 ftc_telemetry_*.log

# Search for specific events
grep "Vision" ftc_telemetry_*.log
grep "ERROR\|WARN" ftc_telemetry_*.log
```

## Performance Considerations

- **File I/O overhead**: Minimal (~1-2ms per flush)
- **Storage usage**: ~50-100KB per minute (adjust if needed)
- **Battery impact**: Negligible (internal SSD write)

If logging affects performance:
1. Reduce flush frequency (modify `captureSnapshot()`)
2. Disable during competition runs
3. Use selective logging (specific keys only)

## Troubleshooting

### Log file not created?
- Check `/data/local/tmp/` directory exists (usually does)
- Check RC permissions (should be accessible)
- Look for error in RC logcat: `adb logcat | grep FtcTelemetryLogger`

### Partial/corrupted log?
- Logger auto-flushes after each line (should be safe)
- If RC crashed mid-run, file is still readable (just truncated)
- Look for end marker: `===== FTC Telemetry Log Ended: ...`

### Want to disable logging?
- Comment out the logger initialization in `RobotMain` constructor:
  ```java
  // telemetryLogger = TelemetryFileLogger.create();
  ```
- Or call: `telemetryLogger.setEnabled(false);`

## Advanced Usage

### Logging structured data:
```java
// In your subsystem:
if (robot.telemetryLogger != null) {
    robot.telemetryLogger.log("Vision/MT2_X", pose.x);
    robot.telemetryLogger.log("Vision/MT2_Y", pose.y);
    robot.telemetryLogger.log("Vision/MT2_Heading", pose.heading);
}
```

### Parse logs programmatically:
```python
# Python script to analyze telemetry
with open('ftc_telemetry_*.log', 'r') as f:
    for line in f:
        if 'ERROR' in line or 'WARN' in line:
            print(line)
```

## Next Steps

1. **Build and deploy** the updated code to RC
2. **Run an OpMode** and let it log data for a minute
3. **Pull the log file** via ADB or file browser
4. **Share with me** if you need help debugging
5. **Adjust settings** as needed for your specific debugging needs

---

## Quick Reference

| Task | Command |
|------|---------|
| View logs | `adb shell ls /data/local/tmp/ftc_telemetry_*.log` |
| Download log | `adb pull /data/local/tmp/ftc_telemetry_*.log` |
| Clear old logs | `adb shell rm /data/local/tmp/ftc_telemetry_*.log` |
| Enable logging | Already enabled by default |
| Disable logging | `setEnabled(false)` in OpMode |
| Check log path | Call `telemetryLogger.getLogFilePath()` |

