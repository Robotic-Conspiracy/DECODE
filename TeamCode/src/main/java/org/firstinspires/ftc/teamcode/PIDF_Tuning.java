package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

@Autonomous
@Config
public class PIDF_Tuning  extends OpMode {
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double targetSpeed = 2000;
    public static double switch_time = 10;
    private Random random = new Random();

    private int Switch = 0;

    private DcMotorEx launcher;
    private ElapsedTime time = new ElapsedTime();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        launcher = hardwareMap.get(DcMotorEx.class, OpmodeConstants.LauncherName);
        time.reset();
    }

    @Override
    public void loop() {
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        telemetry.addData("Velocity", launcher.getVelocity());
        telemetry.addData("target speed", targetSpeed);
        telemetry.addData("Time passed", time.seconds());
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        if(time.seconds() >= switch_time){
            targetSpeed = random.nextInt(2300);
            launcher.setVelocity(targetSpeed);
//            if (Switch == 0) {
//                launcher.setVelocity(-targetSpeed);
//                Switch = 1;
//            } else {
//                launcher.setVelocity(targetSpeed);
//                Switch = 0;
//            }
            time.reset();
        }
        telemetry.update();
    }
}