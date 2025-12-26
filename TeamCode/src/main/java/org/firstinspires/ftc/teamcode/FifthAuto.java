package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

public abstract class FifthAuto extends OpMode {
    protected HardwareRobot robot = new HardwareRobot();
    
    private States state = States.NOT_READY;
    private int timesShot = 0;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    protected String color = "None";
    private final double targetVelocity = 1720;

    @Override
    public void init(){
        String report = robot.init(hardwareMap);
        telemetry.addLine(report);

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(3);
        robot.initVision(aprilTagProcessor);

        if (robot.pod != null) {
            robot.pod.resetPosAndIMU();
        }
        
        telemetry.update();
    }

    @Override
    public void start() {
        waitTimer.reset();
        if (robot.visionPortal != null) {
            while(robot.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !java.lang.Thread.interrupted()){
                telemetry.addLine("Waiting for Camera...");
                telemetry.update();
            }
            ExposureControl exposureControl = robot.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }
        }
    }

    @Override
    public void loop() {
        if (robot.launcher != null) {
            robot.launcher.setVelocity(targetVelocity);
            robot.launcher.setVelocityPIDFCoefficients(203, 1.001, 0.0015, 0.1);
        }
        
        if (robot.bendyServoOne != null) {
            robot.bendyServoOne.setPosition(38/360.0);
        }

        if (robot.pod != null) {
            robot.pod.update();
            telemetry.addData("Pos X", robot.pod.getPosX(DistanceUnit.MM));
            telemetry.addData("Pos Y", robot.pod.getPosY(DistanceUnit.MM));
        }

        if(robot.visionPortal != null && robot.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            handleStateMachine();
        }
    }

    private void handleStateMachine() {
        switch(state) {
            case NOT_READY:
                if (waitTimer.seconds() >= 1) {
                    processNavigation();
                }
                break;
            case SPIN_UP:
                if (feedTimer.seconds() > 3 && robot.launcher != null) {
                    double velocity = robot.launcher.getVelocity();
                    if (velocity >= targetVelocity - 20 && velocity <= targetVelocity + 20) state = States.LAUNCH;
                }
                break;
            case LAUNCH:
                if (robot.leftFeeder != null) robot.leftFeeder.setPower(1);
                if (robot.rightFeeder != null) robot.rightFeeder.setPower(1);
                feedTimer.reset();
                state = States.LAUNCHING;
                break;
            case LAUNCHING:
                if (timesShot <= 4) {
                    if (feedTimer.seconds() > 0.3) {
                        state = States.SPIN_UP;
                        if (robot.leftFeeder != null) robot.leftFeeder.setPower(0);
                        if (robot.rightFeeder != null) robot.rightFeeder.setPower(0);
                        timesShot++;
                        feedTimer.reset();
                    }
                } else {
                    if (robot.leftFeeder != null) robot.leftFeeder.setPower(0);
                    if (robot.rightFeeder != null) robot.rightFeeder.setPower(0);
                    state = States.MOVE;
                }
                break;
            case MOVE:
                executeMoveSequence();
                break;
            case STOP_MOVE:
                robot.setDrivePower(0,0,0,0);
                break;
        }
    }

    private void processNavigation() {
        if (robot.pod == null) return;
        
        double posX = robot.pod.getPosX(DistanceUnit.MM);
        double posY = robot.pod.getPosY(DistanceUnit.MM);

        if (Math.abs(posY) < 25 && Math.abs(posX) < 25 ) {
            move();
        } else {
            List<AprilTagDetection> detections = robot.aprilTagProcessor.getDetections();
            if (detections.isEmpty()) {
                rotate();
            } else {
                handleAprilTagAlignment(detections);
            }
        }
    }

    private void handleAprilTagAlignment(List<AprilTagDetection> detections) {
        int targetId = color.equals("Blue") ? 20 : 24;
        AprilTagDetection target = null;
        
        for (AprilTagDetection d : detections) {
            if (d.id == targetId) target = d;
        }

        if (target != null && target.ftcPose != null) {
            if (Math.abs(target.ftcPose.z) > 0.5) {
                drive(0, 0, Range.clip(target.ftcPose.z * -0.05, -0.15, 0.15));
            } else {
                state = States.SPIN_UP;
                drive(0, 0, 0);
            }
        } else {
            rotate();
        }
    }

    private void executeMoveSequence() {
        if (robot.pod == null) return;
        double posX = robot.pod.getPosX(DistanceUnit.MM);
        double posY = robot.pod.getPosY(DistanceUnit.MM);

        if (Math.abs(posY) < 200 && Math.abs(posX) < 200) {
            move();
        } else if (Math.abs(posX) < 150) {
            strafe();
        } else {
            state = States.STOP_MOVE;
        }
    }

    public abstract void move();
    public abstract void rotate();
    public abstract void strafe();

    public void drive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double fl = (forward - strafe - rotate)/denominator;
        double bl = (forward + strafe - rotate)/denominator;
        double fr = (forward + strafe + rotate)/denominator;
        double br = (forward - strafe + rotate)/denominator;
        robot.setDrivePower(fl, bl, fr, br);
    }

    private enum States { NOT_READY, SPIN_UP, LAUNCH, LAUNCHING, MOVE, STOP_MOVE }
}
