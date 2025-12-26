package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "LM3 Competition Teleop")
public class LM3CompTeleop extends OpMode {

    /* Declare OpMode members. */
    HardwareRobot robot = new HardwareRobot();

    // Gamepad state for edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean lastBumper = false;

    // Constants
    private final double FEED_TIME_SECONDS = 0.20;
    private final double FULL_SPEED = 1.0;
    private final double STOP_SPEED = 0.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 50;

    // Configurable vars
    public static int targetSpeed = 1260;
    public static double targetAngle = 0;

    // Internal state
    private final ElapsedTime Timer = new ElapsedTime();
    private LaunchState launchState = LaunchState.IDLE;
    private Preset selectedPreset = Preset.CUSTOM;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize hardware from the robot class
        String report = robot.init(hardwareMap);
        telemetry.addLine(report);
        
        // Initialize Vision
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(3);
        robot.initVision(aprilTagProcessor);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleVisionControls();
        handleInput();
        handleLauncher();
        handleDrive();
        
        AddTelemetry();
    }

    @Override
    public void stop() {
        if (robot.visionPortal != null) {
            robot.visionPortal.close();
        }
    }

    private void handleVisionControls() {
        if(robot.visionPortal != null && robot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = robot.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }
        }

        List<AprilTagDetection> detections = (robot.aprilTagProcessor != null) ? robot.aprilTagProcessor.getDetections() : null;
        if(detections != null && !detections.isEmpty()){
            for(AprilTagDetection d : detections){
                if((d.id == 24 || d.id == 20) && d.ftcPose != null){
                    telemetry.addData("detected id: ", d.id);
                    telemetry.addData("angle offset ", d.ftcPose.z);
                    if(gamepad1.b && Math.abs(d.ftcPose.z) > 0.5){
                        Drive(0, 0, Range.clip(d.ftcPose.z * -0.05, -0.15, 0.15));
                    }
                }
            }
        }
    }

    private void handleInput() {
        // Edge detection for Dpad Up
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetSpeed += 20 * (gamepad1.x ? 5 : 1);
            selectedPreset = Preset.CUSTOM;
        }
        lastDpadUp = gamepad1.dpad_up;

        // Edge detection for Dpad Down
        if (gamepad1.dpad_down && !lastDpadDown) {
            targetSpeed -= 20 * (gamepad1.x ? 5 : 1);
            selectedPreset = Preset.CUSTOM;
        }
        lastDpadDown = gamepad1.dpad_down;

        // Edge detection for Dpad Right
        if (gamepad1.dpad_right && !lastDpadRight) {
            targetAngle += (gamepad1.x ? 5 : 1);
            selectedPreset = Preset.CUSTOM;
        }
        lastDpadRight = gamepad1.dpad_right;

        // Edge detection for Dpad Left
        if (gamepad1.dpad_left && !lastDpadLeft) {
            targetAngle -= (gamepad1.x ? 5 : 1);
            selectedPreset = Preset.CUSTOM;
        }
        lastDpadLeft = gamepad1.dpad_left;

        targetAngle = Range.clip(targetAngle, SERVO_MINIMUM_POSITION, SERVO_MAXIMUM_POSITION);

        // Edge detection for Y (Cycle Presets)
        if (gamepad1.y && !lastY) {
            cyclePresets();
        }
        lastY = gamepad1.y;
    }

    private void cyclePresets() {
        if (robot.config == null) return;
        
        switch(selectedPreset){
            case CUSTOM:  
                selectedPreset = Preset.GOAL;   
                targetSpeed = robot.config.goalSpeed; 
                targetAngle = robot.config.goalAngle; 
                break;
            case GOAL:    
                selectedPreset = Preset.MIDDLE; 
                targetSpeed = robot.config.middleSpeed; 
                targetAngle = robot.config.middleAngle; 
                break;
            case MIDDLE:  
                selectedPreset = Preset.BACK;   
                targetSpeed = robot.config.backSpeed; 
                targetAngle = robot.config.backAngle; 
                break;
            case BACK:    
                selectedPreset = Preset.JUGGLE; 
                targetSpeed = robot.config.juggleSpeed;  
                targetAngle = robot.config.juggleAngle;  
                break;
            case JUGGLE:  
                selectedPreset = Preset.CUSTOM; 
                break;
        }
    }

    private void handleLauncher() {
        if(robot.launcher != null) robot.launcher.setVelocity(targetSpeed);
        if(robot.bendyServoOne != null) robot.bendyServoOne.setPosition(targetAngle/360);
        
        // Edge detection for bumper
        boolean currentBumper = gamepad1.right_bumper;
        launch(currentBumper && !lastBumper);
        lastBumper = currentBumper;
    }

    private void handleDrive() {
        Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void launch(boolean launchRequested) {
        switch (launchState) {
            case IDLE:
                if (launchRequested) launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                if (robot.launcher != null) {
                    double velocity = robot.launcher.getVelocity();
                    if(velocity >= targetSpeed - 20 && velocity <= targetSpeed + 20) launchState = LaunchState.LAUNCH;
                } else launchState = LaunchState.IDLE;
                break;
            case LAUNCH:
                if (robot.leftFeeder != null) robot.leftFeeder.setPower(FULL_SPEED);
                if (robot.rightFeeder != null) robot.rightFeeder.setPower(FULL_SPEED);
                Timer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if(Timer.seconds() > FEED_TIME_SECONDS){
                    launchState = LaunchState.IDLE;
                    if (robot.leftFeeder != null) robot.leftFeeder.setPower(STOP_SPEED);
                    if (robot.rightFeeder != null) robot.rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    private void Drive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        if(Math.abs(forward) < 0.1) forward = 0;
        if(Math.abs(strafe) < 0.1) strafe = 0;
        if(Math.abs(rotate) < 0.1) rotate = 0;

        double fl = (forward - strafe - rotate) / denominator;
        double bl = (forward + strafe - rotate) / denominator;
        double fr = (forward + strafe + rotate) / denominator;
        double br = (forward - strafe + rotate) / denominator;
        
        robot.setDrivePower(fl, bl, fr, br);
    }

    private void AddTelemetry() {
        telemetry.addData("Preset", selectedPreset);
        telemetry.addData("Velocity (Tgt/Cur)", "%d / %.0f", targetSpeed, (robot.launcher != null) ? robot.launcher.getVelocity() : 0);
        telemetry.addData("Angle (Tgt/Cur)", "%.1f / %.1f", targetAngle, (robot.bendyServoOne != null) ? robot.bendyServoOne.getPosition()*360 : 0);
        telemetry.update();
    }

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private enum Preset { CUSTOM, GOAL, MIDDLE, JUGGLE, BACK }
}
