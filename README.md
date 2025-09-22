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
- [UPSTREAM Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) (contains prebuilt APK files)
- [teamcode (with default README)](TeamCode/src/main/java/org/firstinspires/ftc/teamcode)

## Downloading the Project
To download this Repo please use git clone, we also recommend using ssh with git to verify commits.

# Repo Guidelines
To contribute to our teams codebase there are a few rules:
 - Only add code to [teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode)
	- All other code is illegal to modify except for very rare cases.
 - Only work in your own branch or repo and create pull requests to add to main.
 - Code used in competitions or other important events will be tagged.
 - All pull requests are required to be reviewed by people on the coding team.
  - (if you are on the coding team it is required to be reviewed by someone else)
 - Please push regularly! Regular contributions helps when we do our portfolio, and will show on the GitHub contribution graph when pulled to main.

# Code Structure

Each Module of the Robot should be in its own file and extend the [`RobotModule`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/RobotModule.java) Class, This Class provides the base functions that *should* be used to make the module work. Each Module should have a primary method called `run` that does not have any arguments. For Example a `DriveModule` class might have the following documentation:
```java
public class DriveModule extends RobotModule{

    /**
     * Module run method.
     **/
    public void run(){
        gamepad1 = data.getController(1); 
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        //there is probably more code here
    }
    /** 
    * Makes The Robot Move
    * Requires wheels on the robot
    * All parameters can be negative 
    * 
    * @param x A double representing the desired Forward movement,
    * @param y A double representing the desired Left movement.
    * @param z A double representing the desired Turning to the left.
    **/
    public void drive(double x, double y, double z){
        //There is probability some code here, if there is not we have problems
    }
}
```
Note that this code is only to document how to document things and is not a representation of the actual code of the robot. if there needs to be init code, this can be run in the constructor.

You might have noticed the data class involved here, this class is how all data should be pass through the different modules. you can see this class [here](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/Data.java). To add to the `Data` class, there should be at least a getter method for each variable. If the variable needs to be able to be changed, there should be a setter method as well, otherwise the variable should be assigned in the `init` method. 

The [`Main.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Main.java) file should only have the setup code and other handlers, everything that can reasonably put in a module should be in a module Though there are ovious exceptions for things that are required to be called from an `OPMode` like `telemetry.update()`. 

No matter what code is being added there should be javadoc comments for each method at the minimum. but theoretically the code should be heavily documented. This is **especially** true with trig or other calculations. Though this can be a link to an explanation.

When making a pull request, there should be an explanation of all changes that are being made. It is a good idea to check that all added code is commented properly before making a pull request. The Commit messages should also me clear in what changes are being made, we don't want `added stuff` or `I was told to commit` to be a commit. If needed android studio lets gemini write commit messages for you, though these should be reviewed first and marked as AI generated. It is also generally a good idea to commit each file individually, this can let you write different commit messages for each file, though if the only change is changing a use of a changed function, different commit messages are not needed. ~~Occasionally it can be good to rebase branches with main when the main branch is updated.  this can be done by switching to the main branch and selecting the working branch to rebase.~~ **Don't do this** This can cause issues with duplicate commits, see the commit history for the `meta-changes` branch. 

## Telemetry
Telemetry is still a problem that is being worked on, while we are using [FTC Dashboard](https://acmerobotics.github.io/ftc-dashboard/) the `telemetry` object for some reason is not a linked object, that means that if we add data in one module, and update in another module, it does not properly update with the added data. While we are looking for fixes to this problem we currently have the following cursed code as a stopgap mesure:

https://github.com/Robotic-Conspiracy/DECODE/blob/593de45dcac9032a748d02e57b4ff00bdde2874d/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/RobotModule.java#L25-L33

To use this, please override this method and add the telemetry before the return statement. 

