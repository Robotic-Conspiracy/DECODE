package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LL RED 12 BACK", group = "Autonomous")
public class LL_pickup12red extends LL_pickup12main {
    int backlineAngle = 70;

    @Override
    public void set_starting_pose(){
        starting_pose_x = 81;
        starting_pose_y = 8;
    }

    @Override
    public void set_color(){
        color = "red";
        targetTxOffset = 0.0;
        
        paths.shootPreloadStart = new com.pedropathing.geometry.Pose(81.000, 8.000);
        paths.shootPreloadEnd = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup1Start = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup1End = new com.pedropathing.geometry.Pose(86, 36);
        paths.pickup1Start = new com.pedropathing.geometry.Pose(86, 36);
        paths.pickup1End = new com.pedropathing.geometry.Pose(132, 36);
        paths.shootPickup1Start = new com.pedropathing.geometry.Pose(132, 36);
        paths.shootPickup1End = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup2Start = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup2End = new com.pedropathing.geometry.Pose(92, 55);
        paths.pickUp2Start = new com.pedropathing.geometry.Pose(92, 55);
        paths.pickUp2End = new com.pedropathing.geometry.Pose(132, 55);
        paths.shootPickup2Start = new com.pedropathing.geometry.Pose(132, 60);
        paths.shootPickup2End = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup3Start = new com.pedropathing.geometry.Pose(86, 16);
        paths.prePickup3End = new com.pedropathing.geometry.Pose(86, 84);
        paths.pickup3Start = new com.pedropathing.geometry.Pose(86, 84);
        paths.pickup3End = new com.pedropathing.geometry.Pose(128, 84);
        paths.shootPickup3Start = new com.pedropathing.geometry.Pose(128, 84);
        paths.shootPickup3End = new com.pedropathing.geometry.Pose(86, 84);
        paths.parkStart = new com.pedropathing.geometry.Pose(86, 84);
        paths.parkEnd = new com.pedropathing.geometry.Pose(100, 68);

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
