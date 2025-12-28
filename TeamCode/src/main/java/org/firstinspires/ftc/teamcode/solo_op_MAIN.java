//READ ME
// VARIABLE NAMING CONVENTIONS USED
//PascalCase
//SCREAMING_SNAKE_CASE for FINALS
//
//
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

import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.List;

// to change mapping of buttons ctrl + F search "MAPPING" to Jump to line
@Config

//@TeleOp(name = "Main Solo Op - run color")
public abstract class solo_op_MAIN extends OpMode {
    protected String color = "None";

    abstract void set_color();

    GoBildaPinpointDriver pinpoint;


    // Performance optimization flags
    private boolean lastRightBumper = false;
    private boolean launchRequested = false;
    private long lastFireTime = 0;
    private static final long FIRE_INTERVAL = 500;
    private int loopCounter = 0;
    private double cachedLauncherVelocity = 0;

    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 90;
    public final double INTAKE_POS = .84; // .87MAX
    public final int SPIN_SPEED = -500;
    private double Current_speed = STOP_SPEED;
    double TPS_IN = 0;
    double IN_TARGET_RPM = 0;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    //Drive Motor objects
    private DcMotorEx frontLeftMotor = null;
    private DcMotorEx backLeftMotor = null;
    private DcMotorEx frontRightMotor = null;
    private DcMotorEx backRightMotor = null;
    double velocity = 0;

    AnalogInput floodgate;
    private double X_MOVE = 0;
    private double Y_MOVE = 0;
    private double YAW_MOVE = 0;
    public static double INTAKE_RAMP_POS = .8;
    private final double TPR_6k = 28;
    private final double TPR_1620 = 103.8;
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

    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private Servo LEFT_LAUNCH_SERVO = null;

    private Servo intake_ramp = null;


    //configurable vars
    public static int targetSpeed = 2480;//launch motor speed
    public static double targetAngle = 90 - 38;
    public static int INTAKE_SPEED = 900;

    // other vars and objects
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime Timer2 = new ElapsedTime();
    private final ElapsedTime Timer3 = new ElapsedTime();
    private LaunchState launchState = null;
    private IntakeState intakeState = null;
    private Preset selectedPreset = null;
    private CanLaunch canlaunch = null;

    private final AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;

    private AprilTagDetection targetDetection = null;

    private Servo light1 = null;
    private Servo light2 = null;

    private boolean exposure_set = false;


    @Override
    public void init() {
        set_color();
        launchState = LaunchState.IDLE;
        intakeState = IntakeState.READY;
        selectedPreset = Preset.BACK;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initialize_drive();
        initialize_feeder();
        initialize_launcher();
        initialize_intake();
        initialize_pinpoint();
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
        portal.setProcessorEnabled(aprilTagProcessor, true);


        canlaunch = CanLaunch.ERROR;
    }

    @Override
    public void loop() {
        if (portal.getCameraState() == VisionPortal.CameraState.STREAMING && !exposure_set) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) 16, TimeUnit.MILLISECONDS);
            exposure_set = true;
        }

        //chaing speed
        if (gamepad1.dpadUpWasPressed()) {// MAPPING
            targetSpeed += 20 * (gamepad1.x ? 5 : 1); // MAPPING
        }
        if (gamepad1.dpadDownWasPressed()) {// MAPPING
            targetSpeed -= 20 * (gamepad1.x ? 5 : 1);// MAPPING
        }

        //changing servo rotation
        if (gamepad1.dpadRightWasPressed()) {// MAPPING
            targetAngle += (gamepad1.x ? 5 : 1);// MAPPING
        }
        if (gamepad1.dpadLeftWasPressed()) {// MAPPING
            targetAngle -= (gamepad1.x ? 5 : 1);// MAPPING
        }
        if (targetAngle > SERVO_MAXIMUM_POSITION) {
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION) {
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {// MAPPING
            selectedPreset = Preset.CUSTOM;
        }
        if (gamepad1.yWasPressed()) {// MAPPING
            switch (selectedPreset) {
                case CUSTOM:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1480;
                    targetAngle = 73;
                    break;

                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = 2000;
                    targetAngle = 55;
                    break;

                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = 2480;
                    targetAngle = 48;
                    break;

                case BACK:
                    selectedPreset = Preset.OFF;
                    targetSpeed = 0;
                    targetAngle = 90;
                    break;

                case OFF:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = 1500;
                    targetAngle = 73;
                    break;
            }
        }

        switch (selectedPreset) {
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
        double ON_TIME = .5;
        double OFF_TIME = 1;

        // Cache velocity once per loop to avoid multiple I2C reads
        cachedLauncherVelocity = launcher.getVelocity();

        // Fixed canlaunch logic - previous version always overwrote with NOT_READY
        boolean velocityInRange = (cachedLauncherVelocity >= targetSpeed - 60) && (cachedLauncherVelocity <= targetSpeed + 60);
        boolean velocityNearZero = (cachedLauncherVelocity >= -100) && (cachedLauncherVelocity <= 100);

        if (intakeState == IntakeState.INTAKE) {
            canlaunch = CanLaunch.INTAKE;
        } else if (velocityNearZero || intakeState == IntakeState.NOT_READY) {
            canlaunch = CanLaunch.ERROR;
        } else if (velocityInRange && intakeState == IntakeState.READY) {
            canlaunch = CanLaunch.READY;
        } else {
            canlaunch = CanLaunch.NOT_READY;
        }


        switch (canlaunch) {
            case OFF:
                light2.setPosition(1);
                break;
            case READY:
                light2.setPosition(0.5);
                break;
            case NOT_READY:
                light2.setPosition(0.28);
                break;
            case LAUNCHING:
                light2.setPosition(0.611);
                break;
            case INTAKE:
                light2.setPosition(0.722);
                break;
            case ERROR:
                // Blink: ON for ON_TIME seconds, OFF for (OFF_TIME - ON_TIME) seconds
                double t = Timer3.seconds();
                if (t < ON_TIME) {
                    light2.setPosition(0.28); // ON
                } else if (t < OFF_TIME) {
                    light2.setPosition(0); // OFF
                } else {
                    Timer3.reset(); // restart cycle
                }
                break;
        }


        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection detection = null;
        if (!detections.isEmpty()) {
            for (AprilTagDetection Detection : detections) {
                //if (color.equals("red")) {

                //}

                if (color.equals("red") ? Detection.id == 24 : color.equals("blue") ? Detection.id == 20 : Detection.id == 24 || Detection.id == 20) {
                    detection = Detection;
                    //telemetry.addData("detected id: ", detection.id);
                }
            }
            if (detection != null) {


                //telemetry.addData("angle offset ", detection.ftcPose.z);

                if (gamepad1.right_trigger >= 0.2) {// MAPPING
                    //double z = detection.ftcPose.x;   // left/right
                    double y = detection.ftcPose.y;   // forward/back
                    double x = detection.ftcPose.z;   // up/down


//                double pitch = detection.ftcPose.yaw;   // rotation around vertical axis
                    double yaw = detection.ftcPose.pitch; // rotation around sideways axis
//                double roll = detection.ftcPose.roll;
                    if (Math.abs(x) > 0.5) {
                        X_MOVE = Range.clip(x * -0.05, -1, 1);
                        //Drive(Range.clip(detection.ftcPose.z * -0.05, -1, 1), Range.clip(detection.ftcPose.z * -0.05, -1, 1), Range.clip(detection.ftcPose.z * -0.05, -1, 1));
                    }
                    if (Math.abs(y - 120) > 1) {
                        Y_MOVE = Range.clip((y - 120) * -0.05, -.5, .5);
                    }
                    if (Math.abs(yaw + 25) > 1) {
                        YAW_MOVE = Range.clip((yaw + 25) * 0.05, -.5, .5);
                    }
                    Drive(Y_MOVE, YAW_MOVE, X_MOVE);
                }
                if (gamepad1.b) {

                    double x = detection.ftcPose.z;
                    X_MOVE = Range.clip(x * -0.05, -1, 1);
                    Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, X_MOVE);
                }
            }
        }
        if (targetAngle > SERVO_MAXIMUM_POSITION) {
            targetAngle = SERVO_MAXIMUM_POSITION;
        } else if (targetAngle < SERVO_MINIMUM_POSITION) {
            targetAngle = SERVO_MINIMUM_POSITION;
        }
        launcher.setVelocity(targetSpeed);

        // Throttle telemetry to every 5 loops to reduce lag
        loopCounter++;
        if (loopCounter >= 5) {
            AddTelemetry();
            loopCounter = 0;
        }

        if (!(gamepad1.right_trigger >= 0.2 && detection != null && gamepad1.b)) {
            Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);// MAPPING
        }

        boolean rightBumper = gamepad1.right_bumper;
        long now = System.currentTimeMillis();

        // Fire on initial press
        if (rightBumper && !lastRightBumper) {
            launchRequested = true;
            lastFireTime = now;
        }

        // Continuous fire when held (after interval)
        if (rightBumper && (now - lastFireTime > FIRE_INTERVAL) && launchState == LaunchState.IDLE) {
            launchRequested = true;
            lastFireTime = now;
        }

        lastRightBumper = rightBumper;

        // Always call launch() to process the state machine
        launch();

        //launch(gamepad1.rightBumperWasPressed());// MAPPING
        intake(gamepad1.left_trigger > 0.2, gamepad1.left_bumper); // MAPPING


    }

    private void launch() {
        double FEED_TIME_SECONDS = 0.15;

        switch (launchState) {
            case IDLE:
                if (launchRequested) {
                    launchRequested = false;  // Consume the request
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                double velocity = launcher.getVelocity();
                // normal transition when launcher reaches target RPM
                if (velocity >= targetSpeed - 60 && velocity <= targetSpeed + 60) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                Current_speed = FULL_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                feedTimer.reset(); // start feed duration
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feedTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }
                break;
        }
    }

    private void intake(boolean intakeRequested, boolean spinRequested) {
        // how long the feeders spin reverse when intake starts
        double REV_TIME = 1.0;
        double LAUNCH_POS = 0.61;
        double REV_SPEED = -1.0;

        // If manual spin (hold bumper), go straight to SPIN
        if (spinRequested) {
            intakeState = IntakeState.SPIN;
        } else if (intakeRequested) {
            // If we just started requesting intake, reset the timer to begin the reverse window
            if (intakeState != IntakeState.START_INTAKE && intakeState != IntakeState.INTAKE) {
                Timer2.reset();
            }
            // While within the reverse window, use START_INTAKE
            if (Timer2.seconds() < REV_TIME) {
                intakeState = IntakeState.START_INTAKE;
            } else {
                intakeState = IntakeState.INTAKE;
            }
        } else {
            // No intake requested -> READY and reset timer
            intakeState = IntakeState.READY;
            Timer2.reset();
        }

        switch (intakeState) {
            case READY:
                LEFT_LAUNCH_SERVO.setPosition(targetAngle / 360.0);
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                if (Current_speed == REV_SPEED) {
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }
                break;

            case START_INTAKE:
                // Spin feeders backwards during the reverse window
                Current_speed = REV_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                // keep launcher servo in safe position
                LEFT_LAUNCH_SERVO.setPosition(targetAngle / 360.0);
                break;

            case INTAKE:
                intake_ramp.setPosition(INTAKE_POS);
                IN_RPM = ((intake.getVelocity() / TPR_1620) * 60); // tps / tpr * 60(sec to min)
                IN_TARGET_RPM = ((INTAKE_SPEED / 60.0) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                LEFT_LAUNCH_SERVO.setPosition(0);
                Current_speed = STOP_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                break;

            case SPIN:
                LEFT_LAUNCH_SERVO.setPosition(targetAngle / 360.0);
                IN_RPM = ((intake.getVelocity() / TPR_1620) * 60);
                IN_TARGET_RPM = ((SPIN_SPEED / 60.0) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                intake_ramp.setPosition(LAUNCH_POS);
                break;

            default:
                break;
        }
    }

    private void AddTelemetry() {
        double voltage = floodgate.getVoltage();
        double amps = (voltage / 3.3) * 40.0;
        IN_RPM = ((intake.getVelocity() / TPR_1620) * 60); // tps / tpr * 60(sec to min)

        telemetry.addData("Current (Amps)", "%.2f A", amps);
        telemetry.addData("Current Preset: ", selectedPreset);
        //telemetry.addData("","");
        telemetry.addData("Servo Target Position: ", targetAngle);
        telemetry.addData("L Servo Position: ", LEFT_LAUNCH_SERVO.getPosition() * 360);
        //telemetry.addData("Servo 2 Position: ", intake_ramp.getPosition()*360);
        //telemetry.addData("","");
        telemetry.addData("target velocity", targetSpeed);
        telemetry.addData("current velocity", launcher.getVelocity());
        // telemetry.addData("current Power- launcher", launcher.getPower());
//        telemetry.addData("","");
//        telemetry.addData("intake target RPM", INTAKE_SPEED);
//        telemetry.addData("current INTAKE velocity", IN_RPM);
//        telemetry.addData("current INTAKE power", intake.getPower());
//        telemetry.addData("","");
//        telemetry.addData("front left wheel power", frontLeftMotor.getPower());
//        telemetry.addData("front right wheel power", frontRightMotor.getPower());
//        telemetry.addData("back left wheel power", backLeftMotor.getPower());
//        telemetry.addData("back right wheel power", backRightMotor.getPower());
//        telemetry.addData("","");
//        telemetry.addData("front left wheel speed", FL_RPM);
//        telemetry.addData("front right wheel speed", FR_RPM);
//        telemetry.addData("back left wheel speed", BL_RPM);
//        telemetry.addData("back right wheel speed", BR_RPM);
//        telemetry.addData("","");
        // telemetry.addData("Current (Amps)", "%.2f A", amps);
        //  telemetry.addData("Voltage (Sensor)", "%.2f V", voltage);

        telemetry.update();
    }

    private void initialize_drive() {
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

    private void initialize_launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //launcher motor
        double p = 203;
        double f = 0.1;
        double i = 1.001;
        double d = 0.0015;
        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFT_LAUNCH_SERVO = hardwareMap.get(Servo.class, "left twideler");
    }

    private void initialize_feeder() {
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);//  DIRECTION SETUP
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void initialize_intake() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake_ramp = hardwareMap.get(Servo.class, "intake ramp");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);// DIRECTION SETUP

    }

    private void initialize_pinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        double FORWARD_OFFSET = 1.375;
        double LATERAL_OFFSET = -4.25;


        GoBildaPinpointDriver.GoBildaOdometryPods pods = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        GoBildaPinpointDriver.EncoderDirection forwardDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaPinpointDriver.EncoderDirection lateralDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        pinpoint.setOffsets(FORWARD_OFFSET, LATERAL_OFFSET, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(pods);
        pinpoint.setEncoderDirections(forwardDirection, lateralDirection);
        pinpoint.initialize();
    }

    private void Drive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate / 1.5), 1);
        if (Math.abs(forward) < 0.02) {
            forward = 0;
        }
        if (Math.abs(strafe) < 0.02) {
            strafe = 0;
        }
        if (Math.abs(rotate) < 0.02) {
            rotate = 0;
        }
        if (forward < -1) {
            forward = -1;
        }
        if (strafe < -1) {
            strafe = -1;
        }
        if (rotate < -1) {
            rotate = -1;
        }
        if (forward > 1) {
            forward = 1;
        }
        if (strafe > 1) {
            strafe = 1;
        }
        if (rotate > 1) {
            rotate = 1;
        }
        //rotate = Math.pow(rotate,3);
        double FL_MAX_RPM = 435;
        double FR_MAX_RPM = 435;
        double BL_MAX_RPM = 435;
        double BR_MAX_RPM = 435;
        if (gamepad1.right_stick_button) {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 100;
        } else {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 435;

        }
        double TPS_FL = frontLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_BL = backLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_FR = frontRightMotor.getVelocity(); // default is ticks/sec
        double TPS_BR = backRightMotor.getVelocity(); // default is ticks/sec

        double TPR_435 = 384.5;
        BR_RPM = (TPS_BR * 60) / TPR_435;
        BL_RPM = (TPS_BL * 60) / TPR_435;
        FR_RPM = (TPS_FR * 60) / TPR_435;
        FL_RPM = (TPS_FL * 60) / TPR_435;
        double FL_TARGET_RPM = ((Math.pow(((forward - strafe - rotate) / denominator), 1) * FL_MAX_RPM) * TPR_435) / 60.0;
        double FR_TARGET_RPM = ((Math.pow(((forward + strafe + rotate) / denominator), 1) * BL_MAX_RPM) * TPR_435) / 60.0;
        double BL_TARGET_RPM = ((Math.pow(((forward + strafe - rotate) / denominator), 1) * BR_MAX_RPM) * TPR_435) / 60.0;
        double BR_TARGET_RPM = ((Math.pow(((forward - strafe + rotate) / denominator), 1) * FR_MAX_RPM) * TPR_435) / 60.0;

        //frontLeftMotor.setPower((forward - strafe - rotate)/denominator);  //old method of power, keeping untill velocity is proven to work, may implement as a fallback if encoders are lost ie; wire gets cut/removed
        //backLeftMotor.setPower((forward + strafe - rotate)/denominator);
        //frontRightMotor.setPower((forward + strafe + rotate)/denominator);
        //backRightMotor.setPower((forward - strafe + rotate)/denominator);
        frontLeftMotor.setVelocity(FL_TARGET_RPM);
        backLeftMotor.setVelocity(BL_TARGET_RPM);
        frontRightMotor.setVelocity(FR_TARGET_RPM);
        backRightMotor.setVelocity(BR_TARGET_RPM);

    }

    public AprilTagDetection getTargetDetection() {
        return targetDetection;
    }

    public void setTargetDetection(AprilTagDetection targetDetection) {
        this.targetDetection = targetDetection;
    }

    enum IntakeState {
        SPIN,
        INTAKE,
        READY,
        NOT_READY,
        START_INTAKE
    }

    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    enum Preset {
        CUSTOM,
        GOAL,
        MIDDLE,
        JUGGLE,
        BACK,
        OFF
    }

    enum CanLaunch {
        OFF,//LIGHT will be off,
        READY,//GREEN, RPM in range and AMPS not to high, intake off, everything is correct
        NOT_READY,//RED: if intake or RMP is wrong
        LAUNCHING,//PURPLE : state while launch is happening, it will turn purple
        INTAKE,
        ERROR  //FLASHING RED : error indicates RPM drop and motor bog(power set higher than expected with no reason), voltage criticaly low, and AMP draw over set limit, launch dissabled until AMPS drop below set limit to prevent blowing FUSES
    }
}
