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

@Autonomous(name = "RED 12 BACK", group = "Autonomous")
@Configurable // Panels
public class pickup12red extends pickup12main {
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
    public Pose Path15Start, Path15End;
    // ensure heading variables exist (match base class names)
    public int headShootPreload1;
    public int headShootPreload2;
    public void set_color(){
        String color = "red";
        shootPreloadStart = new Pose(81.000, 8.000);
        shootPreloadEnd = new Pose(86.188, 15.882);
        prePickup1Start = new Pose(86.188, 15.882);
        prePickup1End = new Pose(91.271, 35.576);
        pickup1Start = new Pose(91.271, 35.576);
        pickup1End = new Pose(127.271, 35.365);
        shootPickup1Start = new Pose(127.271, 35.365);
        shootPickup1End = new Pose(85.976, 15.882);
        prePickup2Start = new Pose(86.000, 16.000);
        prePickup2End = new Pose(91.482, 59.929);
        pickUp2Start = new Pose(91.482, 59.929);
        pickUp2End = new Pose(126.635, 59.718);
        shootPickup2Start = new Pose(126.635, 59.718);
        shootPickup2End = new Pose(85.976, 15.882);
        prePickup3Start = new Pose(85.976, 15.882);
        prePickup3End = new Pose(93.176, 84.071);
        pickup3Start = new Pose(93.176, 84.071);
        pickup3End = new Pose(126.635, 83.859);
        shootPickup3Start = new Pose(126.635, 83.859);
        shootPickup3End = new Pose(84.706, 84.282);
        Path15Start = new Pose(84.494, 70.518);
        Path15End = new Pose(118.800, 70.941);
    }}