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

@TeleOp(name = "Main Driver Preset")
public class KylerPreset extends OpMode {

    //Launch servo objects and vars
    private final double FEED_TIME_SECONDS = 0.20;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 50;
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


    //configurable vars
    public static int targetSpeed = 1260;//launch motor speed
    public static double targetAngle = 0;


    // other vars and objects
    private ElapsedTime Timer = new ElapsedTime();
    private LaunchState launchState = null;
    private Preset selectedPreset = null;


    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        selectedPreset = Preset.CUSTOM;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initialize_drive();
        initialize_feeder();
        initialize_launcher();
    }

    @Override
    public void loop() {
        //chaing speed
        if(gamepad1.dpadUpWasPressed()){
            targetSpeed += 20*(gamepad1.x ? 1 : 5);
        }
        if(gamepad1.dpadDownWasPressed()){
            targetSpeed -= 20*(gamepad1.x ? 1 : 5);
        }

        //changing servo rotation
        if(gamepad1.dpadRightWasPressed()){
            targetSpeed += (gamepad1.x ? 1 : 5);
        }
        if(gamepad1.dpadLeftWasPressed()){
            targetSpeed -= (gamepad1.x ? 1 : 5);
        }
        if(gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
            selectedPreset = Preset.CUSTOM;
        }
        if(gamepad1.yWasPressed()){
            switch(selectedPreset){
                case CUSTOM:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1200;
                    targetAngle = 15;
                    break;
                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = 1520;
                    targetAngle = 25;
                    break;
                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = 1920;
                    targetAngle = 35;
                    break;
                case BACK:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1200;
                    targetAngle = 15;
                    break;
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
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    private void initialize_launcher(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        bendyServoOne = hardwareMap.get(Servo.class, "bendy_servo_1");
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
        BACK
    }
}
