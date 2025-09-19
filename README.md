\[THIS README IS IN PROGRESS OF BEING WRITTEN!!\]
## NOTICE
This repository contains the public FTC SDK for the DECODE (2025-2026) competition season as well as as Robotic Conspiracy Team code.

# Welcome!
This GitHub repository contains the source code that is used to build an Android app to control our *FIRST* Tech Challenge competition robot. To use this SDK, download/clone the entire project to your local computer. (Though without a set up robot, good luck with running it)

## Requirements
To use this Android Studio project, you will need Android Studio Ladybug (2024.2) or later.

To program the robot in Blocks or OnBot Java, you do not need Android Studio, but members of the team will look at you funny and odometry will not work right.

## Upstream FTC Links
Here are some of the tutorial and help links from the upstream README file:
- [FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)
- [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html)
- [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html)
- [FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)
- [FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)
- [FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)
- [Samples Folder](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)
- [UPSTREAM Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) (contains prebult APK files)
- [teamcode (with default README)](TeamCode/src/main/java/org/firstinspires/ftc/teamcode)

## Downloading the Project
To download this Repo please use git clone, we also recomend using ssh with git to verify commits.

# Repo Guidelines
To contribute to our teams codebase there are a few rules:
 - Only add code to [teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode)
	- All other code is illegal to modify except for very rare cases.
 - Only work in your own branch or repo and create pull requests to add to main.
 - Code used in competitions or other importent events will be tagged.
 - All pull requests are requiered to be reviewed by people on the coding team.
    - (if you are on the coding team it is requiered to be reviewed by someone else)
 - Please push regularly! Regular contributions helps when we do our portfolio, and will show on the GitHub contribution graph when pulled to main.

# Code Structure
\[In progress, ( I blame @geppp41 ) \]

>⚠️ *All of this is guesses because the outline has not been pushed to any branch yet*

Each Module of the Robot should be in its own file and extend the `RobotModule` Class, This Class provides the base functions that *should* be used to make the module work. Each Moodule should have a primary method (lets call it `run` for now) and have CLEARLY documented requirements for what parameters and other things it requires. For Example a `DriveModule.run` method might have the following documentation:
```java
public class DriveModule extends RobotModule{
    /** 
    * Makes The Robot Move
    * Requires wheels on the robot
    * All parameters can be negitive 
    * 
    * @param x A double representing the desiered Forword movement,
    * @param y A double representing the desiered Left movement.
    * @param z A double representing the desiered Turning to the left.
    **/
    public run(double x, double y, double z){
        //There is probability some code here, if there is not we have problems
    }
}
```
Note that this code is only to document how to document things and is not a representation of the actual code of the robot. 


