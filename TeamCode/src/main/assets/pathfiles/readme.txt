These are the files produced by the pedro pathing visualizer.
We Design our autons around the RED alliance,
V2 - Add it to the list in AutoRegistar and it will mirror XY and make the auto work for blue automatically

File format
-----------
Every file here is saved by the current visualizer (2.0.0) and named <name>.pp.json, which is the
extension the visualizer itself writes. Older builds wrote <name>.pp; those still load, but new
files should keep the .pp.json name.

PPFile reads both versions. The only real difference is that 2.0 files often leave the start
point unnamed, so the loader falls back to "StartPoint" instead of failing.

When you save a path in the visualizer, drop the .pp.json here and point an auton's assetPath at it:

    super.assetPath = "pathfiles/MyAuto.pp.json";
