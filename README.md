# DECODE Game Code for Robotic Conspiracy (FTC 28139)
[![CI (Game-Ready-Code)](https://github.com/Robotic-Conspiracy/DECODE/actions/workflows/ci.yaml/badge.svg?branch=Game-Ready-Code)](https://github.com/Robotic-Conspiracy/DECODE/actions/workflows/ci.yaml) [![CI (master)](https://github.com/Robotic-Conspiracy/DECODE/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/Robotic-Conspiracy/DECODE/actions/workflows/ci.yaml)

## NOTICE
This repository contains the public FTC SDK for the DECODE (2025-2026) competition season as well as as Robotic Conspiracy (28139) Team code.

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

## Code Structure

No matter what code is being added there should be javadoc comments for each method at the minimum, but theoretically the code should be heavily documented. This is **especially** true with trig or other calculations. Though this can be a link to an explanation.

When making a pull request, there should be an explanation of all changes that are being made. It is a good idea to check that all added code is commented properly before making a pull request. The Commit messages should also me clear in what changes are being made, we don't want `added stuff` or `I was told to commit` to be a commit. If needed android studio lets gemini write commit messages for you, though these should be reviewed first and marked as AI generated. It is also generally a good idea to commit each file individually, this can let you write different commit messages for each file, though if the only change is changing a use of a changed function, different commit messages are not needed. ~~Occasionally it can be good to rebase branches with main when the main branch is updated.  this can be done by switching to the main branch and selecting the working branch to rebase.~~ **Don't do this** This can cause issues with duplicate commits, see the commit history for the `meta-changes` branch. 
