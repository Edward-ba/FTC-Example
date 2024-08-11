# Road Runner Quickstart

Check out the [docs](https://rr.brott.dev/docs/v1-0/tuning/).

git clone this project, then open it with android studio, ask Arj for help with git if needed

The only things you should look at for now is Subsystems, Config, ManualRobot, and TeleOpManual 
inside of Teamcode/src/main/java/org.firstinspires.ftc.teamcode, Everything else is pretty much
just library files, and other things. 

The tuning module, messages module, and other files not mention above is for roadrunner, you can
read more at https://rr.brott.dev/ under the 1.0.x section. If you want to see some examples,
check out youtube, also I suggest using MeepMeep to create paths for the robot to follow.

Also, make sure to update the FtcRobotController section whenever a new release comes out at
https://github.com/FIRST-Tech-Challenge/FtcRobotController, just copy over everything in the 
FtcRobotController folder and update the build.dependencies.gradle under gradle scripts to match 
the version on the github

NEVER EVER update the gradle/android sdk verion, no matter how much android studio complains, it
will completely break the build and your code won't work.

To upload code to the robot, connect to the robots wifi (it's normal if it says no internet) and 
run ```adb connect 192.168.43.1:5555``` in a terminal. Build first while on wifi with internet, 
(or have another wifi dongle for multiple wifi's at once) after that you should be fine. 

This will make it so the robot will show up under devices in android studio, before turning off the 
robot or running the program, running ```adb disconnect``` is a good idea as sometimes, it can bug
out and make it impossible to reconnect to the robot without restarting the computer. 

You can also just connect to the robot with a cable.

