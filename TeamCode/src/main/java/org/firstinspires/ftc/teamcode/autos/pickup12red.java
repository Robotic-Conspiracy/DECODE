package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12red extends pickup12main {
    int backlineAngle = 67;
    public void set_color(){
        String color = "red";
        int tagToAim = 24;
        paths.shootPreloadStart = new Pose(81.000, 8.000);
        paths.shootPreloadEnd = new Pose(86.188, 15.882);

        paths.prePickup1Start = new Pose(86.188, 15.882);
        paths.prePickup1End = new Pose(91.271, 35.576);

        paths.pickup1Start = new Pose(91.271, 35.576);
        paths.pickup1End = new Pose(127.271, 35.365);

        paths.shootPickup1Start = new Pose(127.271, 35.365);
        paths.shootPickup1End = new Pose(85.976, 15.882);

        paths.prePickup2Start = new Pose(86.000, 16.000);
        paths.prePickup2End = new Pose(91.482, 59.929);

        paths.pickUp2Start = new Pose(91.482, 59.929);
        paths.pickUp2End = new Pose(126.635, 59.718);

        paths.shootPickup2Start = new Pose(126.635, 59.718);
        paths.shootPickup2End = new Pose(85.976, 15.882);

        paths.prePickup3Start = new Pose(85.976, 15.882);
        paths.prePickup3End = new Pose(93.176, 84.071);

        paths.pickup3Start = new Pose(93.176, 84.071);
        paths.pickup3End = new Pose(126.635, 83.859);

        paths.shootPickup3Start = new Pose(126.635, 83.859);
        paths.shootPickup3End = new Pose(84.706, 84.282);

        paths.parkStart = new Pose(84.494, 70.518);
        paths.parkEnd = new Pose(118.800, 70.941);

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