//READ ME
// VARIABLE NAMING CONVENTIONS USED
//PascalCase
//SCREAMING_SNAKE_CASE for FINALS
//
//
package org.firstinspires.ftc.teamcode.soloOP;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;
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

    // expose a pair of coordinates for subclasses to use; leave undefined (default 0.0) until set at runtime
    protected double back_x;
    protected double back_y;

    abstract void set_color();
    abstract int target_goal_tag();
    abstract void back_line_pos();
    GoBildaPinpointDriver pinpoint;


    // Performance optimization flags
    private boolean lastRightBumper = false;
    private boolean launchRequested = false;
    private long lastFireTime = 0;
    private static final long FIRE_INTERVAL = 500;
    private double cachedLauncherVelocity = 0;
    private double cachedIntakeVelocity = 0;
    private boolean aprilTagProcessorEnabled = true;  // Track processor state to avoid redundant calls

    // Cached pinpoint values to avoid redundant I2C reads
    private double cachedPosX = 0;
    private double cachedPosY = 0;
    private double cachedHeading = 0;

    // Cached light colors to avoid redundant servo writes
    private StatusLightColor cachedLight1Color = null;
    private StatusLightColor cachedLight2Color = null;

    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
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

    public Follower follower;

    AnalogInput floodgate;
    private double X_MOVE = 0;
    private double Y_MOVE = 0;
    private double YAW_MOVE = 0;
    private final double TPR_6k = 28;
    private final double TPR_1620 = 103.8;
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
    public static int INTAKE_SPEED = 1200;
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
    private List<AprilTagDetection> currentDetections = null;

    private Servo light1 = null;
    private Servo light2 = null;
    private boolean exposure_set = false;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(this.getStartPosition());
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

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        pinpoint.update();
        // Cache pinpoint values immediately after update to avoid redundant I2C reads
        cachedPosX = pinpoint.getPosX(DistanceUnit.MM);
        cachedPosY = pinpoint.getPosY(DistanceUnit.MM);
        cachedHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        follower.updatePose();
        moveToPoint(gamepad1.a);
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
        double SERVO_MAXIMUM_POSITION = 90;
        double SERVO_MINIMUM_POSITION = 0;
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
                    targetSpeed = 2440;
                    targetAngle = 52;
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
                setLightColor(light1, StatusLightColor.WHITE);
                break;
            case OFF:
                setLightColor(light1, StatusLightColor.OFF);
                break;
            case GOAL:
                setLightColor(light1, StatusLightColor.ORANGE);
                break;
            case MIDDLE:
                setLightColor(light1, StatusLightColor.AZURE);
                break;
            case JUGGLE:
                setLightColor(light1, StatusLightColor.SAGE);
                break;
            case BACK:
                setLightColor(light1, StatusLightColor.VIOLET);
                break;
        }
        double ON_TIME = .5;
        double OFF_TIME = 1;

        // Cache velocities once per loop to avoid multiple I2C reads
        cachedLauncherVelocity = launcher.getVelocity();
        cachedIntakeVelocity = intake.getVelocity();

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
                setLightColor(light2, StatusLightColor.OFF);
                break;
            case READY:
                setLightColor(light2, StatusLightColor.GREEN);
                break;
            case NOT_READY:
                setLightColor(light2, StatusLightColor.YELLOW);
                break;
            case INTAKE:
                setLightColor(light2, StatusLightColor.RED);
                break;
            case ERROR:
                // Blink: ON for ON_TIME seconds, OFF for (OFF_TIME - ON_TIME) seconds
                double t = Timer3.seconds();
                if (t < ON_TIME) {
                    setLightColor(light2, StatusLightColor.RED); // ON
                } else if (t < OFF_TIME) {
                    setLightColor(light2, StatusLightColor.OFF); // OFF
                } else {
                    Timer3.reset(); // restart cycle
                }
                break;
        }

        // Only enable AprilTag processing when alignment is requested (saves CPU/power)
        boolean alignmentRequested = gamepad1.right_trigger >= 0.2 || gamepad1.b;
        if (alignmentRequested != aprilTagProcessorEnabled) {
            portal.setProcessorEnabled(aprilTagProcessor, alignmentRequested);
            aprilTagProcessorEnabled = alignmentRequested;
        }

        List<AprilTagDetection> detections = alignmentRequested ? aprilTagProcessor.getDetections() : java.util.Collections.emptyList();
        currentDetections = detections;  // Store for telemetry
        AprilTagDetection detection = null;
        if (!detections.isEmpty()) {
            for (AprilTagDetection Detection : detections) {
                if (Detection.id == target_goal_tag()) {
                    detection = Detection;
                }
            }
        }


        if (detection != null) {
            if (gamepad1.right_trigger >= 0.2) {// MAPPING
                    //double z = detection.ftcPose.x;   // left/right
                    double y = detection.ftcPose.y;   // forward/back
                    double x = detection.ftcPose.z;   // up/down


//                double pitch = detection.ftcPose.yaw;   // rotation around vertical axis
                    double yaw = detection.ftcPose.pitch; // rotation around sideways axis
//                double roll = detection.ftcPose.roll;
                    if (Math.abs(x) > 0.5) {
                        X_MOVE = Range.clip(x * -0.05, -0.5, 0.5);
                        //Drive(Range.clip(detection.ftcPose.z * -0.05, -1, 1), Range.clip(detection.ftcPose.z * -0.05, -1, 1), Range.clip(detection.ftcPose.z * -0.05, -1, 1));
                    }
                   //TODO: impliment pedro pathing for moving to position
                    Drive(0, 0, X_MOVE);
                }
                if (gamepad1.b) {

                    double x = detection.ftcPose.z;
                    X_MOVE = Range.clip(x * -0.05, -1, 1);
                    Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, X_MOVE);
                }
        }

        launcher.setVelocity(targetSpeed);

        // Always add telemetry data and update for consistent display
        AddTelemetry();

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

        // Always call launch() to process the launch state machine
        launch();

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
                // Use cached velocity - normal transition when launcher reaches target RPM
                if (cachedLauncherVelocity >= targetSpeed - 60 && cachedLauncherVelocity <= targetSpeed + 60) {
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
        double REV_TIME = 0.2;
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
                IN_RPM = ((cachedIntakeVelocity / TPR_1620) * 60); // tps / tpr * 60(sec to min)
                IN_TARGET_RPM = ((INTAKE_SPEED / 60.0) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                LEFT_LAUNCH_SERVO.setPosition(0);
                Current_speed = STOP_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                break;

            case SPIN:
                LEFT_LAUNCH_SERVO.setPosition(targetAngle / 360.0);
                IN_RPM = ((cachedIntakeVelocity / TPR_1620) * 60);
                IN_TARGET_RPM = ((SPIN_SPEED / 60.0) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                intake_ramp.setPosition(LAUNCH_POS);
                break;

            default:
                break;
        }
    }

    @SuppressLint("DefaultLocale")
    private void AddTelemetry() {
        double voltage = floodgate.getVoltage();
        double amps = (voltage / 3.3) * 40.0;
        IN_RPM = ((cachedIntakeVelocity / TPR_1620) * 60); // tps / tpr * 60(sec to min)

        telemetry.addData("Current (Amps)", "%.2f A", amps);
        telemetry.addData("position x", cachedPosX);
        telemetry.addData("position y", cachedPosY);
        telemetry.addData("angle", cachedHeading);
        telemetry.addData("Current Preset: ", selectedPreset);
        telemetry.addData("Servo Target Position: ", targetAngle);
        telemetry.addData("L Servo Position: ", LEFT_LAUNCH_SERVO.getPosition() * 360);
        telemetry.addData("target velocity", targetSpeed);
        telemetry.addData("current velocity", cachedLauncherVelocity);
        telemetry.addData("gamepad1 left stick x and y", gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.addData("gamepad1 right stick x and y", gamepad1.right_stick_x + " " + gamepad1.right_stick_y);


        // AprilTag telemetry - show all detected tags for consistent display (prevents flickering)

        telemetry.update();
    }

    private void initialize_drive() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");// DRIVE SETUP
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        // Set run mode to RUN_USING_ENCODER for velocity control and proper braking
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        back_line_pos();
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
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        if (Math.abs(forward) < 0.02) {forward = 0;}
        if (Math.abs(strafe) < 0.02) {strafe = 0;}
        if (Math.abs(rotate) < 0.02) {rotate = 0;}
        if (forward < -1) {forward = -1;}
        if (strafe < -1) {strafe = -1;}
        if (rotate < -1) {rotate = -1;}
        if (forward > 1) {forward = 1;}
        if (strafe > 1) {strafe = 1;}
        if (rotate > 1) {rotate = 1;}
        //rotate = Math.pow(rotate,3);
        // If no motion requested, use power(0) so BRAKE engages and return
        if (forward == 0 && strafe == 0 && rotate == 0) {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            return;
        }
        double FL_MAX_RPM = 435;
        double FR_MAX_RPM = 435;
        double BL_MAX_RPM = 435;
        double BR_MAX_RPM = 435;
        if (gamepad1.right_stick_button) {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 100;
        }

        // Note: Removed unused velocity reads (getVelocity calls) that were wasting ~12ms per loop
        double TPR_435 = 384.5;
        double FL_TARGET_RPM = ((Math.pow(((forward - strafe - rotate) / denominator), 1) * FL_MAX_RPM) * TPR_435) / 60.0;
        double FR_TARGET_RPM = ((Math.pow(((forward + strafe + rotate) / denominator), 1) * BL_MAX_RPM) * TPR_435) / 60.0;
        double BL_TARGET_RPM = ((Math.pow(((forward + strafe - rotate) / denominator), 1) * BR_MAX_RPM) * TPR_435) / 60.0;
        double BR_TARGET_RPM = ((Math.pow(((forward - strafe + rotate) / denominator), 1) * FR_MAX_RPM) * TPR_435) / 60.0;
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
        OFF, //LIGHT will be off,
        READY, // RPM in range and AMPS not to high, intake off, everything is correct
        NOT_READY, // if RMP is wrong
        //LAUNCHING,// State while launch is happening, it will turn blue
        INTAKE, // Intake is active
        ERROR  // Error indicates RPM drop and motor bog(power set higher than expected with no reason), voltage criticaly low, and AMP draw over set limit, launch dissabled until AMPS drop below set limit to prevent blowing FUSES
    }

    // GoBilda Status Light colors - servo positions map to colors
    // See: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    enum StatusLightColor {
        OFF(0.0),
        RED(0.280),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0);

        private final double position;

        StatusLightColor(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    // Helper method to set light color - only writes when color changes
    private void setLightColor(Servo light, StatusLightColor color) {
        if (light == light1) {
            if (color != cachedLight1Color) {
                light.setPosition(color.getPosition());
                cachedLight1Color = color;
            }
        } else if (light == light2) {
            if (color != cachedLight2Color) {
                light.setPosition(color.getPosition());
                cachedLight2Color = color;
            }
        } else {
            // Fallback for any other servo
            light.setPosition(color.getPosition());
        }
    }

    public abstract Pose getStartPosition();
    public abstract PathChain pathToTargetPoint(double x, double y, double heading);
    public void moveToPoint(boolean aIsPressed){
        if(aIsPressed){
            switch(selectedPreset){
                case BACK:
                    if(color.equals("blue")){

                    } else if(color.equals("red")) {

                    }
                case GOAL:
                    if(color.equals("blue")){

                    } else if(color.equals("red")) {

                    }
                case MIDDLE:
                    if(color.equals("blue")){

                    } else if(color.equals("red")) {

                    }
                default:
                    break;

            }
        } else {
            follower.pausePathFollowing();
        }
    }
}
