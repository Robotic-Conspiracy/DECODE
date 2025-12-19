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

    //finals (aka vars that wont change)
    private final double FEED_TIME_SECONDS = 0.20;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 50;
    private final String CAMERA_NAME = "Webcam 1";
    private final int CHANGE_SPEED_BY = 20;
    private final int CHANGE_BY_MODIFIER = 5;
    private final double AUTO_ROTATE_GAIN = 0.05;
    private final double AUTO_ROTATE_MAX_POWER = 0.15;
    private final int CAMERA_DECIMATION = 3;
    private final long CAMERA_EXPOSURE_MS = 16;
    private final double AUTO_ROTATE_ANGLE_THRESHOLD = 0.5;
    private final int VELOCITY_TOLERANCE = 20;
    private final double DRIVE_DEADBAND = 0.1;

    // Hardware names
    private final String MOTOR_FRONT_LEFT = "left_front_drive";
    private final String MOTOR_BACK_LEFT = "left_back_drive";
    private final String MOTOR_FRONT_RIGHT = "right_front_drive";
    private final String MOTOR_BACK_RIGHT = "right_back_drive";
    private final String LAUNCHER_NAME = "launcher";
    private final String SERVO_BENDY_NAME = "bendy_servo_1";
    private final String FEEDER_LEFT_NAME = "left_feeder";
    private final String FEEDER_RIGHT_NAME = "right_feeder";

    // AprilTag IDs
    private final int GOAL_TAG_RED = 24;
    private final int GOAL_TAG_BLUE = 20;

    //launcher motor PIDF
    private final double launchP = 203;
    private final double launchI = 1.001;
    private final double launchD = 0.0015;
    private final double launchF = 0.1;

    //Servos
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    //Drive Motor objects
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;


    //nulls? (we will probably use these later)

    private DcMotorEx launcher = null;
    private Servo bendyServoOne = null;


    //configurable vars in FTC dashboard
    public static int targetSpeed = 1720;//launch motor speed
    public static double targetAngle = 38;

    // other vars and objects
    private ElapsedTime Timer = new ElapsedTime();
    private LaunchState launchState = null;
    private Preset selectedPreset = null;

    //Vision
    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private AprilTagDetection targetDetection = null;


    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        selectedPreset = Preset.BACK;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeDrive();
        initializeFeeder();
        initializeLauncher();

        aprilTagProcessor = aprilTagProcessorBuilder.build();

        aprilTagProcessor.setDecimation(CAMERA_DECIMATION);
//        portal = new VisionPortal.Builder()
//                .setCamera(BuiltinCameraDirection.BACK)
//                .addProcessor(aprilTagProcessor)
//                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    @Override
    public void loop() {
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        //changing speed
        if(gamepad1.dpadUpWasPressed()){
            targetSpeed += CHANGE_SPEED_BY * (gamepad1.x ? CHANGE_BY_MODIFIER: 1);
        }
        if(gamepad1.dpadDownWasPressed()){
            targetSpeed -= CHANGE_SPEED_BY * (gamepad1.x ? CHANGE_BY_MODIFIER : 1);
        }

        //changing servo rotation
        if(gamepad1.dpadRightWasPressed()){
            targetAngle += (gamepad1.x ? CHANGE_BY_MODIFIER : 1);
        }
        if(gamepad1.dpadLeftWasPressed()){
            targetAngle -= (gamepad1.x ? CHANGE_BY_MODIFIER : 1);
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
                    targetSpeed = 1080;
                    targetAngle = 14;
                    break;
                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = 1400;
                    targetAngle = 31;
                    break;
                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = 1720;
                    targetAngle = 38;
                    break;
                case BACK:
                    selectedPreset = Preset.JUGGLE;
                    targetSpeed = 500;
                    targetAngle = 8;
                    break;
                case JUGGLE:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1080;
                    targetAngle = 14;
                    break;
            }
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection detection = null;
        if(!detections.isEmpty()){
            for(AprilTagDetection Detection : detections){
                //24 and 20 are the goal tags, so we realy only care about these ones
                if(Detection.id == GOAL_TAG_RED || Detection.id == GOAL_TAG_BLUE){
                    detection = Detection;
                    telemetry.addData("detected id: ", detection.id);
                }
            }
            if(detection != null){
                telemetry.addData("angle offset ", detection.ftcPose.z);
            }
            if(gamepad1.b && detection != null){
                if(Math.abs(detection.ftcPose.z) > AUTO_ROTATE_ANGLE_THRESHOLD) {
                    // Auto-correct robot rotation to align with AprilTag yaw angle
                    Drive(0, 0, Range.clip(detection.ftcPose.z * -AUTO_ROTATE_GAIN, -AUTO_ROTATE_MAX_POWER, AUTO_ROTATE_MAX_POWER));
                }
            }
        }


        if(targetAngle > SERVO_MAXIMUM_POSITION){
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION){
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        launcher.setVelocity(targetSpeed);

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
                if(velocity >= targetSpeed - VELOCITY_TOLERANCE && velocity <= targetSpeed + VELOCITY_TOLERANCE){
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

    private void initializeDrive(){
        frontLeftMotor = hardwareMap.get(DcMotor.class, MOTOR_FRONT_LEFT);
        backLeftMotor = hardwareMap.get(DcMotor.class, MOTOR_BACK_LEFT);
        frontRightMotor = hardwareMap.get(DcMotor.class, MOTOR_FRONT_RIGHT);
        backRightMotor = hardwareMap.get(DcMotor.class, MOTOR_BACK_RIGHT);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    private void initializeLauncher(){
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
        launcher.setVelocityPIDFCoefficients(launchP, launchI, launchD, launchF);
        bendyServoOne = hardwareMap.get(Servo.class, SERVO_BENDY_NAME);
    }
    private void initializeFeeder(){
        leftFeeder = hardwareMap.get(CRServo.class, FEEDER_LEFT_NAME);
        rightFeeder = hardwareMap.get(CRServo.class, FEEDER_RIGHT_NAME);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    private void Drive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        if(Math.abs(forward) < DRIVE_DEADBAND){
            forward = 0;
        }
        if(Math.abs(strafe) < DRIVE_DEADBAND){
            strafe = 0;
        }
        if(Math.abs(rotate) < DRIVE_DEADBAND){
            rotate = 0;
        }

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
