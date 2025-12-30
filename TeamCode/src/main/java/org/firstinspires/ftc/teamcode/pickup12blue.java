package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends pickup12main {
    public Pose shootPreloadStart, shootPreloadEnd;
    public Pose prePickup1Start, prePickup1End;
    public Pose pickup1Start, pickup1End;
    public Pose shootPickup1Start, shootPickup1End;
    public Pose prePickup2Start, prePickup2End;
    public Pose pickUp2Start, pickUp2End;
    public Pose shootPickup2Start, shootPickup2End;
    public Pose prePickup3Start, prePickup3End;
    public Pose pickup3Start, pickup3End;
    public Pose shootPickup3Start, shootPickup3End;
    public Pose parkStart, parkEnd;
    public  int headShootPreload1;
    public  int headShootPreload2;
    public int headprePickup11;
    public int headprePickup12;
    public int headpickup1;
    public int headshootPickup11;
    public int headshootPickup12;
    public int headprePickup21;
    public int headprePickup22;
    public int headpickUp2;
    public int headshootPickup21;
    public int headshootPickup22;
    public int headprePickup31;
    public int headprePickup32;
    public int headpickup3;

    public int headshootPickup31;
    public int headshootPickup32;
    public int headpark;

    public void set_color(){
        String color = "blue";
        shootPreloadStart = new Pose(63.000, 8.000);
        shootPreloadEnd = new Pose(57.812, 15.882);
        headShootPreload1 = 90;
        headShootPreload2 = 113;
        prePickup1Start = new Pose(57.812, 15.882);
        prePickup1End = new Pose(52.729, 35.576);
        headprePickup11 = 113;
        headprePickup12 = 180;
        pickup1Start = new Pose(52.729, 35.576);
        pickup1End = new Pose(16.729, 35.365);
        headpickup1 = 180;
        shootPickup1Start = new Pose(16.729, 35.365);
        shootPickup1End = new Pose(58.024, 15.882);
        headshootPickup11 = 180;
        headshootPickup12 = 113;
        prePickup2Start = new Pose(58, 16);
        prePickup2End = new Pose(52.518, 59.929);
        headprePickup21 = 113;
        headprePickup22 = 180;
        pickUp2Start = new Pose(52.518, 59.929);
        pickUp2End = new Pose(17.365, 59.718);
        headpickUp2 = 180;
        shootPickup2Start = new Pose(17.365, 59.718);
        shootPickup2End = new Pose(58.024, 15.882);
        headshootPickup21 = 180;
        headshootPickup22 = 113;
        prePickup3Start = new Pose(58.024, 15.882);
        prePickup3End = new Pose(50.824, 84.071);
        headprePickup31 = 113;
        headprePickup32 = 180;
        pickup3Start = new Pose(50.824, 84.071);
        pickup3End = new Pose(17.365, 83.859);
        headpickup3 = 180;
        shootPickup3Start = new Pose(17.365, 83.859);
        shootPickup3End = new Pose(59.294, 84.282);
        headshootPickup31 = 180;
        headshootPickup32 = 135;
        parkStart = new Pose(59.506, 70.518);
        parkEnd = new Pose(25.200, 70.941);
        headpark = 180;



    }








}
