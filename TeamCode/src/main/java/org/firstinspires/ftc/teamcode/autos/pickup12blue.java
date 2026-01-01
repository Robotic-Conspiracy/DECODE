package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends pickup12main {
    int backlineAngle = 111;
    public void set_color(){
        String color = "blue";
        int tagToAim = 20;
        paths.shootPreloadStart = new Pose(63.000, 8.000);
        paths.shootPreloadEnd = new Pose(58, 16);

        paths.prePickup1Start = new Pose(58, 16);
        paths.prePickup1End = new Pose(58, 36);

        paths.pickup1Start = new Pose(58, 36);
        paths.pickup1End = new Pose(15, 36);

        paths.shootPickup1Start = new Pose(15, 36);
        paths.shootPickup1End = new Pose(58, 16);

        paths.prePickup2Start = new Pose(58, 16);
        paths.prePickup2End = new Pose(52, 55);

        paths.pickUp2Start = new Pose(52, 55);
        paths.pickUp2End = new Pose(18, 55);

        paths.shootPickup2Start = new Pose(20, 60);
        paths.shootPickup2End = new Pose(58, 16);

        paths.prePickup3Start = new Pose(58, 16);
        paths.prePickup3End = new Pose(58, 84);

        paths.pickup3Start = new Pose(58, 84);
        paths.pickup3End = new Pose(18, 84);

        paths.shootPickup3Start = new Pose(18, 84);
        paths.shootPickup3End = new Pose(58, 84);

        paths.parkStart = new Pose(58, 84);
        paths.parkEnd = new Pose(25, 72);

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