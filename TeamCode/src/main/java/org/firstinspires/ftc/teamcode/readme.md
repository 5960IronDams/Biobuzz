howdy, 

to debug this robot, Connect to the 5960-RC wifi
Status page : http://192.168.43.1:8080/?page=connection.html&pop=true
PanelsDashboard : http://192.168.43.1:8080 (Telemetry + Field + Configurables tabs)

We use Pedro pathing for pathing and ivy for commands
Currently planning solverslib installion for Linear interpolation tables
and Command Controller for triggers

Live tuning: public static fields on @Configurable classes
(Intake, PositionalServo, AimAtGoal, PanelsFieldRenderer) are editable
in Panels > Configurables while the OpMode runs. Field pose comes from
PanelsFieldRenderer (Panels > Field, BIOBUZZ background).

Creating Autos:
WE ONLY CREATE RED SIDE AUTO! blue will be automatically created when you make the Blue link and change the extends below!
Create path files and put them in assets/pathfiles 
Copy the example auto rename (example : Red_YourNewAuto) and tell PPFile to load your new path file of the same name
Then double check in init that your start position is loaded
then edit the autoroutine() to your liking. this is what does all the movements and actions.
copy the Blue_ example auto and change it from "extends Red_exampleAuto" to "extends Red_YourNewAuto"
