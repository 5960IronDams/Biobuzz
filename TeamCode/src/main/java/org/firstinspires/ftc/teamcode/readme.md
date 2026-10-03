howdy, 

to debug this robot, Connect to the 5960-RC wifi
Status page : http://192.168.43.1:8080/?page=connection.html&pop=true
PanelsDashboard : http://192.168.43.1:8080 (Telemetry + Field + Configurables tabs)
Pedro tuning : http://192.168.43.1:10158/

We use Pedro pathing for pathing and Solverslib for commands
solverslib  for Linear interpolation tables and WPILIB compatibility in general
solverslib also handles our subsystemBase
made our own solverslib Command Controller for triggered button binding

Live tuning: public static fields on @Configurable classes
(Intake, PositionalServo, AimAtGoal, PanelsFieldRenderer) are editable
in Panels > Configurables while the OpMode runs. Field pose comes from
PanelsFieldRenderer (Panels > Field, BIOBUZZ background).

Creating Autos:
WE ONLY CREATE RED SIDE AUTO! blue will be automatically created When Autos are added to "AutoRegistrar" list!
Create path files and put them in assets/pathfiles 
Copy the example auto rename (example : Red_YourNewAuto) and tell PPFile to load your new path file of the same name
Then double check in init that your start position is loaded
then edit the autoroutine() to your liking. this is what does all the movements and actions.
Add your Red Auton To the AutoRegistrar List and it will make 2 auton op modes one with prefix Red_ and one with prefix Blue_

//TODO: check this

