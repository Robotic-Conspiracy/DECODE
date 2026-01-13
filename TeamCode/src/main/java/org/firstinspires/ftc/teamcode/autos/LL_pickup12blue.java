package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LL BLUE 12 BACK", group = "Autonomous")
public class LL_pickup12blue extends LL_pickup12main {
    int backlineAngle = 109;

    @Override
    public void set_starting_pose(){
        starting_pose_x = 63;
        starting_pose_y = 8;
    }

    @Override
    public void set_color(){
        color = "blue";
        targetTxOffset = 0.0;
        
        paths.shootPreloadStart = new com.pedropathing.geometry.Pose(63.000, 8.000);
        paths.shootPreloadEnd = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup1Start = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup1End = new com.pedropathing.geometry.Pose(58, 36);
        paths.pickup1Start = new com.pedropathing.geometry.Pose(58, 36);
        paths.pickup1End = new com.pedropathing.geometry.Pose(12, 36);
        paths.shootPickup1Start = new com.pedropathing.geometry.Pose(12, 36);
        paths.shootPickup1End = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup2Start = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup2End = new com.pedropathing.geometry.Pose(52, 55);
        paths.pickUp2Start = new com.pedropathing.geometry.Pose(52, 55);
        paths.pickUp2End = new com.pedropathing.geometry.Pose(12, 55);
        paths.shootPickup2Start = new com.pedropathing.geometry.Pose(12, 60);
        paths.shootPickup2End = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup3Start = new com.pedropathing.geometry.Pose(58, 16);
        paths.prePickup3End = new com.pedropathing.geometry.Pose(58, 84);
        paths.pickup3Start = new com.pedropathing.geometry.Pose(58, 84);
        paths.pickup3End = new com.pedropathing.geometry.Pose(16, 84);
        paths.shootPickup3Start = new com.pedropathing.geometry.Pose(16, 84);
        paths.shootPickup3End = new com.pedropathing.geometry.Pose(58, 84);
        paths.parkStart = new com.pedropathing.geometry.Pose(58, 84);
        paths.parkEnd = new com.pedropathing.geometry.Pose(44, 68);

        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;
        paths.headprePickup11 = backlineAngle;
        paths.headprePickup12 = 180;
        paths.headpickup1 = 180;
        paths.headshootPickup11 = 180;
        paths.headshootPickup12 = backlineAngle;
        paths.headprePickup21 = backlineAngle;
        paths.headprePickup22 = 180;
        paths.headpickUp2 = 180;
        paths.headshootPickup21 = 180;
        paths.headshootPickup22 = backlineAngle;
        paths.headprePickup31 = backlineAngle;
        paths.headprePickup32 = 180;
        paths.headpickup3 = 180;
        paths.headshootPickup31 = 180;
        paths.headshootPickup32 = 135;
        paths.headpark = 0;
    }
}
