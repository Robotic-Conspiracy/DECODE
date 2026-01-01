package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12red extends pickup12main {
    int backlineAngle = 67;
    @Override
    public void set_starting_pose(){
        starting_pose_x = 81;
        starting_pose_y = 8;

        //follower.setStartingPose(new Pose(81, 8, Math.toRadians(90)));
    }
    public void set_color(){
        String color = "red";
        int tagToAim = 24;
        // Mirrored from blue across x = 72 -> x' = 144 - x
        paths.shootPreloadStart = new Pose(81.000, 8.000); // from blue 63.000
        paths.shootPreloadEnd = new Pose(86, 16); // from blue 58

        paths.prePickup1Start = new Pose(86, 16); // from blue 58
        paths.prePickup1End = new Pose(86, 36); // from blue 58

        paths.pickup1Start = new Pose(86, 36); // from blue 58
        paths.pickup1End = new Pose(136, 36); // from blue 8

        paths.shootPickup1Start = new Pose(136, 36); // from blue 8
        paths.shootPickup1End = new Pose(86, 16); // from blue 58

        paths.prePickup2Start = new Pose(86, 16); // from blue 58
        paths.prePickup2End = new Pose(92, 55); // from blue 52

        paths.pickUp2Start = new Pose(92, 55); // from blue 52
        paths.pickUp2End = new Pose(136, 55); // from blue 8

        paths.shootPickup2Start = new Pose(136, 60); // from blue 8 (y was 60)
        paths.shootPickup2End = new Pose(86, 16); // from blue 58

        paths.prePickup3Start = new Pose(86, 16); // from blue 58
        paths.prePickup3End = new Pose(86, 84); // from blue 58

        paths.pickup3Start = new Pose(86, 84); // from blue 58
        paths.pickup3End = new Pose(132, 84); // from blue 12

        paths.shootPickup3Start = new Pose(132, 84); // from blue 12
        paths.shootPickup3End = new Pose(86, 84); // from blue 58

        paths.parkStart = new Pose(86, 84); // from blue 58
        paths.parkEnd = new Pose(100, 68); // from blue 25

        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;

        paths.headprePickup11 = backlineAngle;
        paths.headprePickup12 = 0;

        paths.headpickup1 = 0;

        paths.headshootPickup11 = 0;
        paths.headshootPickup12 = backlineAngle;

        paths.headprePickup21 = backlineAngle;
        paths.headprePickup22 = 0;

        paths.headpickUp2 = 0;

        paths.headshootPickup21 = 0;
        paths.headshootPickup22 = backlineAngle;

        paths.headprePickup31 = backlineAngle;
        paths.headprePickup32 = 0;

        paths.headpickup3 = 0;

        paths.headshootPickup31 = 0;
        paths.headshootPickup32 = 45;

        paths.headpark = 180;
    }
}