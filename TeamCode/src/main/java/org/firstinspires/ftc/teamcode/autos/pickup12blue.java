package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends pickup12main {
    int backlineAngle = 113;
    public void set_color(){
        String color = "blue";
        int tagToAim = 20;
        paths.shootPreloadStart = new Pose(63.000, 8.000);
        paths.shootPreloadEnd = new Pose(57.812, 15.882);

        paths.prePickup1Start = new Pose(57.812, 15.882);
        paths.prePickup1End = new Pose(52.729, 35.576);

        paths.pickup1Start = new Pose(52.729, 35.576);
        paths.pickup1End = new Pose(16.729, 35.365);

        paths.shootPickup1Start = new Pose(16.729, 35.365);
        paths.shootPickup1End = new Pose(58.024, 15.882);

        paths.prePickup2Start = new Pose(58, 16);
        paths.prePickup2End = new Pose(52.518, 59.929);

        paths.pickUp2Start = new Pose(52.518, 59.929);
        paths.pickUp2End = new Pose(17.365, 59.718);

        paths.shootPickup2Start = new Pose(17.365, 59.718);
        paths.shootPickup2End = new Pose(58.024, 15.882);

        paths.prePickup3Start = new Pose(58.024, 15.882);
        paths.prePickup3End = new Pose(50.824, 84.071);

        paths.pickup3Start = new Pose(50.824, 84.071);
        paths.pickup3End = new Pose(17.365, 83.859);

        paths.shootPickup3Start = new Pose(17.365, 83.859);
        paths.shootPickup3End = new Pose(59.294, 84.282);

        paths.parkStart = new Pose(59.506, 70.518);
        paths.parkEnd = new Pose(25.200, 70.941);

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
        paths.headshootPickup32 = 135;//mid angle

        paths.headpark = 0;
    }
}