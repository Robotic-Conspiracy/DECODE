//package org.firstinspires.ftc.teamcode.autos;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous(name = "BLUE 6 BACK", group = "Autonomous")
//@Configurable // Panels
//public class pickup6blue extends pickup6main {
//    int backlineAngle = 110;
//    public void set_starting_pose(){
//        starting_pose_x = 63;
//        starting_pose_y = 8;
//        starting_pose_heading = 90;
//        //follower.setStartingPose(new Pose(63, 8, Math.toRadians(90)));
//    }
//    public void set_color(){
//        String color = "blue";
//        int tagToAim = 20;
//        paths.shootPreloadStart = new Pose(63.000, 8.000);
//        paths.shootPreloadEnd = new Pose(58, 16);
//
//        paths.prePickup1Start = new Pose(58, 16);
//        paths.prePickup1End = new Pose(12, 16);
//
//        paths.pickup1Start = new Pose(12, 16);
//        paths.pickup1End = new Pose(12, 36);
//
//        paths.shootPickup1Start = new Pose(12, 36);
//        paths.shootPickup1End = new Pose(8, 5);
////
//        paths.prePickup2Start = new Pose(58, 16);
//        paths.prePickup2End = new Pose(52, 55);
//
//        paths.pickUp2Start = new Pose(52, 55);
//        paths.pickUp2End = new Pose(12, 55);
//
//        paths.shootPickup2Start = new Pose(12, 60);
//        paths.shootPickup2End = new Pose(58, 16);
//
//        paths.prePickup3Start = new Pose(58, 16);
//        paths.prePickup3End = new Pose(58, 84);
//
//        paths.pickup3Start = new Pose(58, 84);
//        paths.pickup3End = new Pose(16, 84);
//
//        paths.shootPickup3Start = new Pose(16, 84);
//        paths.shootPickup3End = new Pose(58, 84);
//
//        paths.parkStart = new Pose(58, 84);
//        paths.parkEnd = new Pose(44, 68);
////
//        paths.headShootPreload1 = 90;
//        paths.headShootPreload2 = backlineAngle;
//
//        paths.headprePickup11 = backlineAngle;
//        paths.headprePickup12 = 222;
//
//        paths.headpickup1 = 250;
//
//        paths.headshootPickup11 = 180;
//        paths.headshootPickup12 = backlineAngle;
////
//        paths.headprePickup21 = backlineAngle;
//        paths.headprePickup22 = 180;
//
//        paths.headpickUp2 = 180;
//
//        paths.headshootPickup21 = 180;
//        paths.headshootPickup22 = backlineAngle;
//
//        paths.headprePickup31 = backlineAngle;
//        paths.headprePickup32 = 180;
//
//        paths.headpickup3 = 180;
//
//        paths.headshootPickup31 = 180;
//        paths.headshootPickup32 = 135;//mid angle
//
//        paths.headpark = 0;
//    }
//}