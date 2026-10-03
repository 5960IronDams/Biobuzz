# ADB Commands for Telemetry Logging

Quick reference for accessing telemetry logs from the REV Control Hub via ADB (Android Debug Bridge).

## Prerequisites
- Android SDK installed (includes ADB)
- RC connected via USB and Developer Mode enabled
- PowerShell terminal

## View Log Files on RC

```powershell
# List all telemetry log files
adb shell ls -lh /data/local/tmp/ftc_telemetry_*.log

# View latest log file (last 50 lines)
adb shell tail -50 /data/local/tmp/ftc_telemetry_*.log

# View specific log file (replace timestamp)
adb shell cat /data/local/tmp/ftc_telemetry_2024-12-29_14-30-45.log
```

## Download Log Files to Computer

```powershell
# Download latest telemetry log
adb pull /data/local/tmp/ftc_telemetry_*.log .

# Download specific log (replace timestamp)
adb pull /data/local/tmp/ftc_telemetry_2024-12-29_14-30-45.log

# Download to specific folder
adb pull /data/local/tmp/ftc_telemetry_*.log "C:\Logs\FTC"
```

## Search/Filter Logs

```powershell
# Search for specific events (while on RC)
adb shell grep "VISION" /data/local/tmp/ftc_telemetry_*.log

# Search for errors
adb shell grep "ERROR\|WARN" /data/local/tmp/ftc_telemetry_*.log

# After pulling to computer - search for specific text
Select-String "ERROR" ftc_telemetry_*.log

# Count lines in log
adb shell wc -l /data/local/tmp/ftc_telemetry_*.log
```

## Cleanup

```powershell
# Remove all telemetry logs from RC
adb shell rm /data/local/tmp/ftc_telemetry_*.log

# Remove logs older than a certain pattern
adb shell rm /data/local/tmp/ftc_telemetry_2024-12-29_*.log
```

## View Logcat (for logger debug messages)

```powershell
# View real-time logcat (filter for telemetry logger)
adb logcat | Select-String "FtcTelemetryLogger"

# View last 100 logcat lines
adb logcat -d | tail -100

# Save logcat to file
adb logcat -d > logcat_backup.txt
```

## Full Workflow Example

```powershell
# 1. Connect RC and verify
adb devices

# 2. List available logs
adb shell ls -lh /data/local/tmp/ftc_telemetry_*.log

# 3. Pull latest log (adjust timestamp as needed)
adb pull /data/local/tmp/ftc_telemetry_2024-12-29_14-30-45.log

# 4. View log in PowerShell
Get-Content ftc_telemetry_2024-12-29_14-30-45.log | tail -50

# 5. Search for specific event
Select-String "Vision" ftc_telemetry_2024-12-29_14-30-45.log

# 6. Open in Notepad for viewing
notepad ftc_telemetry_2024-12-29_14-30-45.log
```

## Troubleshooting

```powershell
# Check if RC is properly connected
adb devices -l

# Verify logger has permissions to write to /data/local/tmp
adb shell ls -ld /data/local/tmp

# Check available space on RC
adb shell df -h /data/local/tmp

# View ADB logs for connection issues
adb logcat | Select-String "TELEMETRY"
```

## PowerShell Aliases (Optional - add to profile)

```powershell
# Add these to your PowerShell profile ($PROFILE) for quick access:

function Get-FtcLogs {
    adb pull "/data/local/tmp/ftc_telemetry_*.log" "."
}

function Show-FtcLogs {
    adb shell ls -lh /data/local/tmp/ftc_telemetry_*.log
}

function Tail-FtcLog {
    adb shell tail -100 /data/local/tmp/ftc_telemetry_*.log
}

function Find-InFtcLog {
    param([string]$pattern)
    adb shell grep "$pattern" /data/local/tmp/ftc_telemetry_*.log
}

function Clear-FtcLogs {
    adb shell rm /data/local/tmp/ftc_telemetry_*.log
}

# Usage:
# Get-FtcLogs
# Show-FtcLogs
# Tail-FtcLog
# Find-InFtcLog "ERROR"
# Clear-FtcLogs
```

