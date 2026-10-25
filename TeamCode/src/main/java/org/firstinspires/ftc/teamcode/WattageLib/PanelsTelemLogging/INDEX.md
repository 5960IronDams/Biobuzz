# FTC Telemetry Logging System - Complete Index

## 🚀 Start Here

**New to telemetry logging?**  
→ Read: [TELEMETRY_LOGGING_README.md](TELEMETRY_LOGGING_README.md)

**Want the full guide?**  
→ Read: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md)

**Ready to deploy?**  
→ Check: [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md)

**Need ADB commands?**  
→ Reference: [ADB_TELEMETRY_REFERENCE.md](ADB_TELEMETRY_REFERENCE.md)

---

## 📚 Documentation Files

| File | Purpose | Audience |
|------|---------|----------|
| [TELEMETRY_LOGGING_README.md](TELEMETRY_LOGGING_README.md) | Quick start overview | Everyone |
| [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md) | Complete user guide with examples | Developers, Team Leads |
| [ADB_TELEMETRY_REFERENCE.md](ADB_TELEMETRY_REFERENCE.md) | ADB commands and PowerShell helpers | Developers retrieving logs |
| [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) | Technical architecture details | Advanced developers |
| [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md) | Pre-deployment checklist | Before releasing to RC |
| [DELIVERABLES.md](DELIVERABLES.md) | Complete list of what was delivered | Project overview |
| **INDEX.md** | This file - navigation guide | Everyone |

---

## 💻 Source Code Files

### Core Logging System
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/
├── TelemetryFileLogger.java
│   ├── Main logging engine
│   ├── File I/O and timestamp management
│   ├── Auto-flush for crash safety
│   └── Key Methods:
│       ├── create() - Factory method
│       ├── captureSnapshot(telemetry) - Log telemetry frame
│       ├── log(key, value) - Manual logging
│       ├── setEnabled(boolean) - Toggle logging
│       └── close() - Finalize logging
│
├── TelemetrySnapshot.java
│   ├── Advanced telemetry extraction
│   ├── Reflection-based data access
│   ├── Map and list formatting
│   └── Key Methods:
│       ├── captureDetail(telemetry) - Extract all data
│       └── extractAllData(telemetry) - Get as Map
│
└── TelemetryLoggerStatus.java
    ├── Status verification helper
    ├── Display logger state in Panels
    └── Key Methods:
        └── report() - Show status in telemetry
```

### Integration Point
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
└── RobotMain.java (MODIFIED)
    ├── Added: import TelemetryFileLogger
    ├── Added: public telemetryLogger field
    ├── Modified: Constructor - initializes logger
    └── Modified: flushTelemetry() - captures before update
```

### Example Code
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/
└── ExampleTelemetryLogging.java
    ├── ExampleTelemetryLoggingAuto - Autonomous example
    ├── ConditionalLoggingExample - Enabling/disabling
    ├── SubsystemDebugLoggingExample - Component logging
    └── ErrorLoggingExample - Error/warning logging
```

---

## 🎯 Common Tasks

### I want to...

#### Deploy the logging system
1. Open project in Android Studio
2. Build: Build → Make Project
3. Deploy: Run → Run 'TeamCode'
4. **No additional setup needed** - logging is automatic

#### Pull a telemetry log
```powershell
adb pull /data/local/tmp/ftc_telemetry_*.log
```
→ See: [ADB_TELEMETRY_REFERENCE.md](ADB_TELEMETRY_REFERENCE.md)

#### Search a log for specific events
```powershell
Select-String "Vision" ftc_telemetry_*.log
```
→ See: [ADB_TELEMETRY_REFERENCE.md](ADB_TELEMETRY_REFERENCE.md)

#### Add manual logging to my OpMode
```java
if (robot.telemetryLogger != null) {
    robot.telemetryLogger.log("MY_EVENT", "Something");
}
```
→ See: [ExampleTelemetryLogging.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/ExampleTelemetryLogging.java)

#### Disable logging for a run
```java
robot.telemetryLogger.setEnabled(false);
```
→ See: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md)

#### Check if logging is working
```java
new TelemetryLoggerStatus(robot).report();
```
→ See: [ExampleTelemetryLogging.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/ExampleTelemetryLogging.java)

#### Share logs with support team
1. Pull log: `adb pull /data/local/tmp/ftc_telemetry_*.log`
2. Attach to GitHub issue or email
3. Include: robot state, when issues occurred, what you were testing
→ See: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md#sharing-logs-with-support)

#### Understand log file format
See: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md#log-file-format)

#### Troubleshoot logging problems
See: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md#troubleshooting)

#### Check system performance impact
See: [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md#performance-considerations)

---

## 📋 File Structure

```
Biobuzz-5960/
│
├── [📄 DOCUMENTATION FILES]
│   ├── TELEMETRY_LOGGING_README.md       ← Quick start
│   ├── TELEMETRY_LOGGING_GUIDE.md        ← Full guide
│   ├── ADB_TELEMETRY_REFERENCE.md        ← Commands
│   ├── IMPLEMENTATION_SUMMARY.md         ← Architecture
│   ├── VERIFICATION_CHECKLIST.md         ← Pre-deploy
│   ├── DELIVERABLES.md                   ← What's included
│   └── INDEX.md                          ← This file
│
├── [📂 TEAM CODE]
│   └── TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│       │
│       ├── [🔧 TELEMETRY SYSTEM - NEW]
│       │   └── utils/
│       │       ├── TelemetryFileLogger.java      ← Core logger
│       │       ├── TelemetrySnapshot.java        ← Data extraction
│       │       └── TelemetryLoggerStatus.java    ← Status helper
│       │
│       ├── [📖 EXAMPLES - NEW]
│       │   └── examples/
│       │       └── ExampleTelemetryLogging.java
│       │
│       └── [🤖 MAIN CODE - MODIFIED]
│           └── RobotMain.java                    ← Logger integrated
│
└── [📊 OTHER]
    ├── build.gradle
    ├── settings.gradle
    ├── gradle.properties
    └── ... (other project files)
```

---

## 🔄 Quick Reference

### Log File Location
```
RC Storage: /data/local/tmp/ftc_telemetry_YYYY-MM-DD_HH-mm-ss.log
Pull via:  adb pull /data/local/tmp/ftc_telemetry_*.log
```

### Log File Format
```
[XXXXXX ms] [key1 = value1; key2 = value2; ...]
[XXXXXX ms] [key1 = value1; key2 = value2; ...]
```

### Key Methods
```java
// Automatic (no code needed)
flushTelemetry()  // Automatically captures

// Manual logging
telemetryLogger.log(key, value)

// Status check
new TelemetryLoggerStatus(robot).report()

// Control logging
telemetryLogger.setEnabled(false/true)
```

### ADB Commands
```powershell
# List logs
adb shell ls /data/local/tmp/ftc_telemetry_*.log

# Pull log
adb pull /data/local/tmp/ftc_telemetry_*.log

# Search log
adb shell grep "keyword" /data/local/tmp/ftc_telemetry_*.log

# View last N lines
adb shell tail -50 /data/local/tmp/ftc_telemetry_*.log
```

---

## ✅ Verification

- [x] All code compiles without errors
- [x] All documentation complete
- [x] All examples working
- [x] ADB commands tested
- [x] Ready for production deployment

---

## 📞 Need Help?

1. **Getting started?** → [TELEMETRY_LOGGING_README.md](TELEMETRY_LOGGING_README.md)
2. **How do I use it?** → [TELEMETRY_LOGGING_GUIDE.md](TELEMETRY_LOGGING_GUIDE.md)
3. **What commands do I use?** → [ADB_TELEMETRY_REFERENCE.md](ADB_TELEMETRY_REFERENCE.md)
4. **How does it work?** → [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)
5. **Can I run it?** → [VERIFICATION_CHECKLIST.md](VERIFICATION_CHECKLIST.md)
6. **Show me code examples** → [ExampleTelemetryLogging.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/ExampleTelemetryLogging.java)

---

**Status**: ✅ Ready for Production  
**Version**: 1.0  
**Date**: 2026-09-29  
