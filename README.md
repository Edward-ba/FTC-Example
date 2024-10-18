# Readme

Check out the [roadrunner docs](https://rr.brott.dev/docs/v1-0/tuning/).

git clone this project, then open it with android studio, ask Arj for help with git if needed

Also, make sure to update the FtcRobotController section whenever a new release comes out at
https://github.com/FIRST-Tech-Challenge/FtcRobotController, copy over everything in the 
FtcRobotController folder and update the build.dependencies.gradle under gradle scripts to match 
the version on the GitHub

NEVER EVER click APG upgrade, no matter how much Android Studio complains, it
will completely break the build and your code won't work.

You should only look at Subsystems, Config, ManualRobot, and TeleOpManual 
inside of Teamcode/src/main/java/org.firstinspires.ftc.teamcode, Everything else is
just library files and Roadrunner. 

The tuning module, messages module, and other files not mentioned above are for Roadrunner, you can
read more at https://rr.brott.dev/ under the 1.0.x section. If you'd like to see some examples,
you can check out YouTube, also I suggest using MeepMeep to create paths for the robot to follow.

To upload code to the robot, connect to the robot's wifi (it's normal if it says no internet) and 
run ```./adb connect 192.168.43.1:5555``` in a terminal/git bash. Build first while on wifi with the internet, 
(or have another wifi dongle for multiple wifi's at once) after that you should be fine. 

This will make it so the robot will show up under devices in Android Studio, before turning off the 
robot or running the program, running ```./adb disconnect``` is a good idea as sometimes, it can bug
out and make it impossible to reconnect to the robot without restarting the computer. 

You can also connect to the robot with a USB-C cable.
