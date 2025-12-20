package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;
// to change mapping of buttons ctrl + F search "MAPPING" to Jump to line
@Config
@TeleOp(name = "Solo Op -sams code")
public class sams_teleop extends OpMode {

    //Launch servo objects and vars
    private final double FEED_TIME_SECONDS = 0.30;
    private final double REV_TIME = 0.5;  //how long the feeders spin reversse when intake starts
    private final double REV_SPEED = -1.0;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = -300;
    private final double SERVO_MAXIMUM_POSITION = 300;
    private final double INTAKE_POS = 0;
    private final double LAUNCH_POS = 95;
    private final int spin_speed = -500;
    private double Current_speed = STOP_SPEED;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    //Drive Motor objects
    private DcMotorEx frontLeftMotor = null;
    private DcMotorEx backLeftMotor = null;
    private DcMotorEx frontRightMotor = null;
    private DcMotorEx backRightMotor = null;
    AnalogInput floodgate;
    //private AnalogInput floodgateCurrent;
    private static double FL_MAX_RPM = 400;
    private static double FR_MAX_RPM = 400;
    private static double BL_MAX_RPM = 400;
    private static double BR_MAX_RPM = 400;
    private final double TPR_435 = 384.5;
    private final double TPR_6k = 28;
    private final double TPR_1640 = 145.6;
//    //double TPS_FL = frontLeftMotor.getVelocity(); // default is ticks/sec
//    //double TPS_BL = backLeftMotor.getVelocity(); // default is ticks/sec
//    //double TPS_FR = frontRightMotor.getVelocity(); // default is ticks/sec
//    double TPS_BR = backRightMotor.getVelocity(); // default is ticks/sec
//    double BR_RPM = (TPS_BR * 60) / TPR_435;
//    double BL_RPM = (TPS_BL * 60) / TPR_435;
//    double FR_RPM = (TPS_FR * 60) / TPR_435;
//    double FL_RPM = (TPS_FL * 60) / TPR_435;
//    private  double FL_TARGET_RPM = (0 * TPR_435) / 60.0;
//
//    private double FR_TARGET_RPM = (0 * TPR_435) / 60.0;
//    private double BL_TARGET_RPM = (0 * TPR_435) / 60.0;
//    private double BR_TARGET_RPM = (0 * TPR_435) / 60.0;
    double FL_RPM = 0;
    double FR_RPM = 0;
    double BL_RPM = 0;
    double BR_RPM = 0;
    double IN_RPM = 0;

    //launcher motor
    private final double P = 203;
    private final double I = 1.001;
    private final double D = 0.0015;
    private final double F = 0.1;

    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private Servo LEFT_LAUNCH_SERVO = null;
    
    private Servo intake_ramp = null;


    //configurable vars
    public static int targetSpeed = 0;//launch motor speed
    public static double targetAngle = 38;
    public static int intake_speed = 1400;
    // other vars and objects
    private ElapsedTime Timer = new ElapsedTime();
    private ElapsedTime Timer2 = new ElapsedTime();
    private LaunchState launchState = null;
    private IntakeState intakeState = null;
    private Preset selectedPreset = null;
    private CanLaunch canlaunch = null;

    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private AprilTagDetection targetDetection = null;

    private Servo light1 = null;
    private Servo light2 = null;
    


    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        intakeState = IntakeState.READY;
        selectedPreset = Preset.BACK;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initialize_drive();
        initialize_feeder();
        initialize_launcher();
        initialize_intake();
        light1 = hardwareMap.get(Servo.class, "preset light");
        light2 = hardwareMap.get(Servo.class, "launch light");
        floodgate = hardwareMap.get(AnalogInput.class, "floodgate");
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
        canlaunch = CanLaunch.READY;
    }

    @Override
    public void loop() {
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) 10, TimeUnit.MILLISECONDS);
        }
        //chaing speed
        if(gamepad1.dpadUpWasPressed()){// MAPPING
            targetSpeed += 20*(gamepad1.x ? 5 : 1); // MAPPING
        }
        if(gamepad1.dpadDownWasPressed()){// MAPPING
            targetSpeed -= 20*(gamepad1.x ? 5 : 1);// MAPPING
        }

        //changing servo rotation
        if(gamepad1.dpadRightWasPressed()){// MAPPING
            targetAngle += (gamepad1.x ? 5 : 1);// MAPPING
        }
        if(gamepad1.dpadLeftWasPressed()){// MAPPING
            targetAngle -= (gamepad1.x ? 5 : 1);// MAPPING
        }
        if (targetAngle > SERVO_MAXIMUM_POSITION){
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION){
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        if(gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {// MAPPING
            selectedPreset = Preset.CUSTOM;
        }
        if(gamepad1.yWasPressed()){// MAPPING
            switch(selectedPreset){
                case CUSTOM:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1080;
                    targetAngle = 90-14;
                    break;
                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = 1400;
                    targetAngle = 90-31;
                    break;
                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = 1720;
                    targetAngle = 90-38;
                    break;
                case BACK:
                    selectedPreset = Preset.JUGGLE;
                    targetSpeed = 500;
                    targetAngle = 90-20;
                    break;
                case JUGGLE:
                    selectedPreset = Preset.OFF;
                    targetSpeed = 0;
                    targetAngle = 90-0;
                    break;

                case OFF:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1080;
                    targetAngle = 90-14;
                    break;
            }
        }

        switch (selectedPreset){
            case CUSTOM:
                light1.setPosition(1);
                break;
            case OFF:
                light1.setPosition(0.277);
                break;
            case GOAL:
                light1.setPosition(0.5);
                break;
            case MIDDLE:
                light1.setPosition(0.388);
                break;
            case JUGGLE:
                light1.setPosition(0.555);
                break;
            case BACK:
                light1.setPosition(0.722);
                break;
        }
        switch (canlaunch){
            case OFF:
                light2.setPosition(0);
                break;
            case READY:
                light2.setPosition(0.277);
                break;
            case NOT_READY:
                light2.setPosition(0.5);
                break;
            case LAUNCHING:
                light2.setPosition(0.388);
                break;
            case ERROR:
                light2.setPosition(0.555);
                break;
        }


        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection detection = null;
        if(!detections.isEmpty()){
            for(AprilTagDetection Detection : detections){
                if(Detection.id == 24 || Detection.id == 20){
                    detection = Detection;
                    telemetry.addData("detected id: ", detection.id);
                }
            }
            if(detection != null){
                telemetry.addData("angle offset ", detection.ftcPose.z);
            }
            if(gamepad1.right_trigger > 0.5 && detection != null){// MAPPING
                if(Math.abs(detection.ftcPose.z) > 0.5) {
                    Drive(0, 0, Range.clip(detection.ftcPose.z * -0.25, -0.50, 0.50));//made -0.05 -0.10 to maybe increse speed, if broken revert value
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
        Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);// MAPPING
        LEFT_LAUNCH_SERVO.setPosition(targetAngle/360);
        launch(gamepad1.rightBumperWasPressed());// MAPPING
        intake(gamepad1.left_trigger > 0.5, gamepad1.left_bumper); // MAPING


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
                Current_speed = FULL_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                Timer.reset();
                launchState = LaunchState.LAUNCHING;

                break;
            case LAUNCHING:
                if(Timer.seconds() > FEED_TIME_SECONDS){
                    launchState = LaunchState.IDLE;
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }
                break;

        }
    }
    private void intake(boolean intakeRequested, boolean spinRequested) {
        intakeState = IntakeState.READY;
        intakeState = spinRequested ? IntakeState.SPIN : (intakeRequested ? (Timer2.seconds() > REV_TIME/2 ?IntakeState.START_INTAKE : IntakeState.INTAKE): IntakeState.READY);
        switch  (intakeState) {
            case READY:
                intake_ramp.setPosition(LAUNCH_POS/360);
                intake.setVelocity(0);
                if(Current_speed == REV_SPEED){
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }

                break;

            case START_INTAKE:
                if (Current_speed != REV_SPEED) {
                    Timer2.reset();
                    Current_speed = REV_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                    //intake_ramp.setPosition(INTAKE_POS / 360);
                    }
                if(Timer2.seconds() > REV_TIME/2) {
                    intakeState = IntakeState.INTAKE;
                }
                break;


            case INTAKE:
                intake_ramp.setPosition(INTAKE_POS / 360);
                double TPS_IN = intake.getVelocity();
                IN_RPM = (TPS_IN * 60) / TPR_1640;
                double IN_TARGET_RPM = (intake_speed * TPR_1640);
                intake.setVelocity(IN_TARGET_RPM);

                if(Timer2.seconds() > REV_TIME){
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }
                break;

            case SPIN:
                intake.setVelocity(spin_speed);
                intake_ramp.setPosition(LAUNCH_POS/360);
                break;

        }
    }
    private void AddTelemetry() {
        double voltage = floodgate.getVoltage();
        double amps = (voltage / 3.3) * 40.0;
        double TPS_IN = intake.getVelocity();
        IN_RPM = (TPS_IN * 60) / TPR_1640;
        telemetry.addData("Current Preset: ", selectedPreset);
        telemetry.addData("","");
        telemetry.addData("Servo Target Position: ", targetAngle);
        telemetry.addData("L Servo Position: ", LEFT_LAUNCH_SERVO.getPosition()*360);
        telemetry.addData("Servo 2 Position: ", intake_ramp.getPosition()*360);
        telemetry.addData("","");
        telemetry.addData("target velocity", targetSpeed);
        telemetry.addData("current velocity", launcher.getVelocity());
        telemetry.addData("current Power- launcher", launcher.getPower());
        telemetry.addData("","");
        telemetry.addData("intake target RPM", intake_speed);
        telemetry.addData("current INTAKE velocity", IN_RPM);
        telemetry.addData("current INTAKE power", intake.getPower());
        telemetry.addData("","");
        telemetry.addData("front left wheel power", frontLeftMotor.getPower());
        telemetry.addData("front right wheel power", frontRightMotor.getPower());
        telemetry.addData("back left wheel power", backLeftMotor.getPower());
        telemetry.addData("back right wheel power", backRightMotor.getPower());
        telemetry.addData("","");
        telemetry.addData("front left wheel speed", FL_RPM);
        telemetry.addData("front right wheel speed", FR_RPM);
        telemetry.addData("back left wheel speed", BL_RPM);
        telemetry.addData("back right wheel speed", BR_RPM);
        telemetry.addData("","");
        telemetry.addData("Current (Amps)", "%.2f A", amps);
        telemetry.addData("Voltage (Sensor)", "%.2f V", voltage);

        telemetry.update();
    }

    private void initialize_drive(){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");// DRIVE SETUP
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// DRIVE SETUP
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);// DRIVE SETUP
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);//
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    private void initialize_launcher(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        LEFT_LAUNCH_SERVO = hardwareMap.get(Servo.class, "left twideler");
    }
    private void initialize_feeder(){
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);//  DIRECTION SETUP
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    private void initialize_intake(){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake_ramp = hardwareMap.get(Servo.class, "intake ramp");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);// DIRECTION SETUP

    }


    private void Drive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        if(Math.abs(forward) > 0.02){
            forward += 0.03;
        }
        if(Math.abs(strafe) < 0.02){
            strafe += 0.03;
        }
        if(Math.abs(rotate) < 0.02){
            rotate += 0.03;
            rotate = rotate/2;
            
        }
        if (gamepad1.right_stick_button) {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 100;
        }
        else{
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 400;

        }
        double TPS_FL = frontLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_BL = backLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_FR = frontRightMotor.getVelocity(); // default is ticks/sec
        double TPS_BR = backRightMotor.getVelocity(); // default is ticks/sec

        BR_RPM = (TPS_BR * 60) / TPR_435;
        BL_RPM = (TPS_BL * 60) / TPR_435;
        FR_RPM = (TPS_FR * 60) / TPR_435;
        FL_RPM = (TPS_FL * 60) / TPR_435;
        double FL_TARGET_RPM = ((Math.pow(((forward - strafe - rotate)/denominator),3)*FL_MAX_RPM)   * TPR_435) / 60.0;
        double FR_TARGET_RPM = ((Math.pow(((forward + strafe + rotate)/denominator),3)*BL_MAX_RPM) * TPR_435) / 60.0;
        double BL_TARGET_RPM = ((Math.pow(((forward + strafe - rotate)/denominator),3)*BR_MAX_RPM) * TPR_435) / 60.0;
        double BR_TARGET_RPM = ((Math.pow(((forward - strafe + rotate)/denominator),3)*FR_MAX_RPM) * TPR_435) / 60.0;

        //frontLeftMotor.setPower((forward - strafe - rotate)/denominator);  //old method of power, keeping untill velocity is proven to work, may implement as a fallback if encoders are lost ie; wire gets cut/removed
        //backLeftMotor.setPower((forward + strafe - rotate)/denominator);
        //frontRightMotor.setPower((forward + strafe + rotate)/denominator);
        //backRightMotor.setPower((forward - strafe + rotate)/denominator);
        frontLeftMotor.setVelocity(FL_TARGET_RPM);
        backLeftMotor.setVelocity(BL_TARGET_RPM);
        frontRightMotor.setVelocity(FR_TARGET_RPM);
        backRightMotor.setVelocity(BR_TARGET_RPM);

    }
    private enum IntakeState {
        SPIN,
        INTAKE,
        READY,
        NOT_READY,
        START_INTAKE
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
        BACK,
        OFF
    }
    private enum CanLaunch {
        OFF,//LIGHT will be off,
        READY,//GREEN, RPM in range and AMPS not to high, intake off, everything is correct
        NOT_READY,//RED: if intake or RMP is wrong
        LAUNCHING,//PURPLE : state while launch is happening, it will turn purple
        ERROR  //FLASHING RED : error indecates RPM drop and motor bog(power set higher than expected with no reason), voltage criticaly low, and AMP draw over set limit, launch dissabled until AMPS drop below set limit to prevent blowing FUSES
    }
}
