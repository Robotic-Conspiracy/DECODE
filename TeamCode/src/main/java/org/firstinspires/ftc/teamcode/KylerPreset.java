package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Main Driver Preset")
public class KylerPreset extends OpMode {

    //Launch servo objects and vars
    private final double FEED_TIME_SECONDS = 0.20;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 50;
    private final double KICKER_MINIMUM_POSITION = 110;
    private final double KICKER_MAXIMUM_POSITION = 0;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    //Drive Motor objects
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    //launcher motor
    private final double P = 203;
    private final double I = 1.001;
    private final double D = 0.0015;
    private final double F = 0.1;

    private DcMotorEx launcher = null;
    private Servo bendyServoOne = null;
    private Servo artifactKicker = null;


    //configurable vars
    public static int targetSpeed = 1260;//launch motor speed
    public static double targetAngle = 0;


    // other vars and objects
    private ElapsedTime Timer = new ElapsedTime();
    private LaunchState launchState = null;
    private Preset selectedPreset = null;

    private CRServo grabbyServo1;
    private CRServo grabbyServo2;
    private CRServo grabbyServo3;

    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private AprilTagDetection targetDetection = null;


    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        selectedPreset = Preset.CUSTOM;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initialize_drive();
        initialize_feeder();
        initialize_launcher();

        grabbyServo1 = hardwareMap.get(CRServo.class, "servo 1");
        grabbyServo2 = hardwareMap.get(CRServo.class, "servo 2");
        grabbyServo3 = hardwareMap.get(CRServo.class, "servo 3");
        grabbyServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        grabbyServo2.setDirection(DcMotorSimple.Direction.REVERSE);
        grabbyServo3.setDirection(DcMotorSimple.Direction.FORWARD);

        aprilTagProcessor = aprilTagProcessorBuilder.build();

        aprilTagProcessor.setDecimation(3);
//        portal = new VisionPortal.Builder()
//                .setCamera(BuiltinCameraDirection.BACK)
//                .addProcessor(aprilTagProcessor)
//                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    @Override
    public void loop() {
        //chaing speed
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) 1, TimeUnit.MILLISECONDS);
        }
        if(gamepad1.dpadUpWasPressed()){
            targetSpeed += 20*(gamepad1.x ? 5 : 1);
        }
        if(gamepad1.dpadDownWasPressed()){
            targetSpeed -= 20*(gamepad1.x ? 5 : 1);
        }

        //changing servo rotation
        if(gamepad1.dpadRightWasPressed()){
            targetAngle += (gamepad1.x ? 5 : 1);
        }
        if(gamepad1.dpadLeftWasPressed()){
            targetAngle -= (gamepad1.x ? 5 : 1);
        }
        if (targetAngle > SERVO_MAXIMUM_POSITION){
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION){
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        if(gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
            selectedPreset = Preset.CUSTOM;
        }
        if(gamepad1.yWasPressed()){
            switch(selectedPreset){
                case CUSTOM:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1200;
                    targetAngle = 18;
                    break;
                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = 1520;
                    targetAngle = 28;
                    break;
                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = 1920;
                    targetAngle = 38;
                    break;
                case BACK:
                    selectedPreset = Preset.JUGGLE;
                    targetSpeed = 600;
                    targetAngle = 8;
                    break;
                case JUGGLE:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1200;
                    targetAngle = 15;
                    break;

            }
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection detection = null;
        if(!detections.isEmpty()) {
            for (AprilTagDetection Detection : detections) {
                if (Detection.id == 24 || Detection.id == 20) {
                    detection = Detection;
                    telemetry.addData("detected id: ", detection.id);
                }
            }
            if (gamepad1.b && detection != null) {
                if (Math.abs(detection.ftcPose.z) > 0.75) {
                    Drive(0, 0, Range.clip(detection.ftcPose.z * -0.05, -0.15, 0.15));
                }
            }
        }
        if(targetAngle > SERVO_MAXIMUM_POSITION){
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION){
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        if(gamepad1.right_trigger >=0.5){
            grabbyServo2.setPower(1);
            grabbyServo1.setPower(1);
            grabbyServo3.setPower(1);
        } else {
            grabbyServo2.setPower(0);
            grabbyServo1.setPower(0);
            grabbyServo3.setPower(0);
        }
        launcher.setVelocity(targetSpeed);
        if(gamepad1.leftBumperWasPressed()){
            artifactKicker.setPosition(KICKER_MAXIMUM_POSITION/360);
        } else {
            artifactKicker.setPosition(KICKER_MINIMUM_POSITION/360);
        }

        AddTelemetry();
        Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        bendyServoOne.setPosition(targetAngle/360);
        launch(gamepad1.rightBumperWasPressed());


    }

    private void launch(boolean launchRequested) {
        switch  (launchState) {
            case IDLE:
                launchState = launchRequested ? LaunchState.SPIN_UP : launchState;
                break;
            case SPIN_UP:
                double velocity = launcher.getVelocity();
                if(velocity >= targetSpeed -20 && velocity <= targetSpeed +20){
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                Timer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if(Timer.seconds() > FEED_TIME_SECONDS){
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;

        }
    }

    private void AddTelemetry() {
        telemetry.addData("Current Preset: ", selectedPreset);
        telemetry.addData("Servo Target Position: ", targetAngle);
        telemetry.addData("Servo Position: ", bendyServoOne.getPosition()*360);
        telemetry.addData("target velocity", targetSpeed);
        telemetry.addData("current velocity", launcher.getVelocity());
        telemetry.addData("front left wheel power", frontLeftMotor.getPower());
        telemetry.addData("front right wheel power", frontRightMotor.getPower());
        telemetry.addData("back left wheel power", backLeftMotor.getPower());
        telemetry.addData("back right wheel power", backRightMotor.getPower());
        telemetry.update();
    }

    private void initialize_drive(){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    private void initialize_launcher(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        bendyServoOne = hardwareMap.get(Servo.class, "bendy_servo_1");
        artifactKicker = hardwareMap.get(Servo.class, "artifact_kicker");
    }
    private void initialize_feeder(){
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    private void Drive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        frontLeftMotor.setPower((forward - strafe - rotate)/denominator);
        backLeftMotor.setPower((forward + strafe - rotate)/denominator);
        frontRightMotor.setPower((forward + strafe + rotate)/denominator);
        backRightMotor.setPower((forward - strafe + rotate)/denominator);

    }


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private enum Preset {
        CUSTOM,
        GOAL,
        MIDDLE,
        JUGGLE,
        BACK
    }
}
