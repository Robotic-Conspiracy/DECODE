package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config
public class PIDF_Tuning  extends OpMode {
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double switch_time = 10;

    private int Switch = 0;

    private DcMotorEx launcher;
    private ElapsedTime time = new ElapsedTime();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        time.reset();
    }

    @Override
    public void loop() {
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        telemetry.addData("Velocity", launcher.getVelocity());
        telemetry.addData("Time passed", time.seconds());
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        if(time.seconds() >= switch_time){
            if (Switch == 0) {
                launcher.setVelocity(-2000);
                Switch = 1;
            } else {
                launcher.setVelocity(2000);
                Switch = 0;
            }
            time.reset();
        }
        telemetry.update();
    }
}
