package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotShootAndDrive", group = "StarterBot")
@Disabled
@Config
public class StarterBotShootAndDrive extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    public static int targetSpeed = 1250;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize robot hardware and get report
        String report = robot.init(hardwareMap);
        telemetry.addLine(report);
        
        if (robot.launcher != null) {
            robot.launcher.setVelocityPIDFCoefficients(500, 0.2, 0, 0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        
        if(gamepad1.dpadUpWasPressed()) targetSpeed += 10;
        if(gamepad1.dpadDownWasPressed()) targetSpeed -= 10;
        
        if (robot.launcher != null) {
            robot.launcher.setVelocity(targetSpeed);
        }
        
        launch(gamepad1.rightBumperWasPressed());

        telemetry.addData("target speed", targetSpeed);
        telemetry.addData("current speed", (robot.launcher != null) ? robot.launcher.getVelocity() : 0);
        telemetry.update();
    }

    void mecanumDrive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double fl = (forward + strafe + rotate) / denominator;
        double fr = (forward - strafe - rotate) / denominator;
        double bl = (forward - strafe + rotate) / denominator;
        double br = (forward + strafe - rotate) / denominator;

        robot.setDrivePower(fl, bl, fr, br);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                if (robot.launcher != null) {
                    if (robot.launcher.getVelocity() >= targetSpeed - 40) launchState = LaunchState.LAUNCH;
                } else launchState = LaunchState.IDLE;
                break;
            case LAUNCH:
                if (robot.leftFeeder != null) robot.leftFeeder.setPower(FULL_SPEED);
                if (robot.rightFeeder != null) robot.rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if(feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    if (robot.leftFeeder != null) robot.leftFeeder.setPower(STOP_SPEED);
                    if (robot.rightFeeder != null) robot.rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
