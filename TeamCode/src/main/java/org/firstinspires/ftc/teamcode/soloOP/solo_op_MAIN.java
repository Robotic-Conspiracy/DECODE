package org.firstinspires.ftc.teamcode.soloOP;
import org.firstinspires.ftc.teamcode.OpmodeConstants;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
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
import org.firstinspires.ftc.teamcode.StaticCommunism;
import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.List;
import java.util.function.Supplier;

// to change mapping of buttons ctrl + F search "MAPPING" to Jump to line

@Configurable
//@TeleOp(name = "Main Solo Op - run color")
public abstract class solo_op_MAIN extends OpMode {
    protected String color = "None";
    public long now = System.currentTimeMillis();
    abstract void set_color();
    abstract int target_goal_tag();
    abstract int set_backline_angle();
    GoBildaPinpointDriver pinpoint;
    private Supplier<PathChain> pathToBack;
    public boolean AutoMove;


    // Performance optimization flags
    private boolean lastRightBumper = false;
    private boolean launchRequested = false;
    private long lastFireTime = 0;
    private static final long FIRE_INTERVAL = 300; // milliseconds between launches
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

    // AprilTag alignment PD controller variables
    private double lastAlignmentError = 0;
    private long lastAlignmentTime = 0;
    private double filteredDerivative = 0;  // Low-pass filtered derivative to reduce noise
    private static final double ALIGNMENT_DEADBAND = 4.0;  // Degrees - balanced accuracy vs speed
    private static final double ALIGNMENT_P_GAIN = 0.025;  // Proportional gain - balanced for speed without oscillation
    private static final double ALIGNMENT_D_GAIN = 0.05;   // Derivative gain - higher to dampen oscillation and prevent overshoot
    private static final double ALIGNMENT_MAX_POWER = 0.3;  // Max rotation power - fast but controllable
    private static final double ALIGNMENT_MIN_POWER = 0.06; // Minimum power to overcome static friction
    private static final double DERIVATIVE_FILTER_ALPHA = 0.4; // Moderate filtering for stability

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

    AnalogInput floodgate;
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
    public static int backlineSpeed = OpmodeConstants.TeleopBacklineSpeed;
    public static double backlineAngle = OpmodeConstants.TeleopBacklineAngle;
    public static int midSpeed = OpmodeConstants.TeleopMidlineSpeed;
    public static double midAngle = OpmodeConstants.TeleopMidAngle;
    public static int goalSpeed = OpmodeConstants.TeleopGoalSpeed;
    public static double goalAngle = OpmodeConstants.TeleopGoalAngle;

    //configurable vars
    public static int targetSpeed = backlineSpeed;//launch motor speed
    public static double targetAngle = backlineAngle;
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

    private Servo stoppy_servo;

    private Follower follower;
    public Pose backlinePosition;
    public int backlineAimAngle;
    public PathChain path;
    private boolean breakModeActive;
    private TelemetryManager panelsTelemetry;


    @Override
    public void init() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StaticCommunism.pose);
        backlinePosition = set_backline_position();
        backlineAimAngle = set_backline_angle();
        Drawing.init();
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
        light1 = hardwareMap.get(Servo.class, OpmodeConstants.PresetLightName);
        light2 = hardwareMap.get(Servo.class, OpmodeConstants.AimLightName);
        stoppy_servo = hardwareMap.get(Servo.class, OpmodeConstants.IntakeStopperName);
        floodgate = hardwareMap.get(AnalogInput.class, OpmodeConstants.FloodgateName);
        pathToBack = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, backlinePosition)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(backlineAimAngle), 0.8))
                .build();
        aprilTagProcessor = aprilTagProcessorBuilder.build();

        aprilTagProcessor.setDecimation(3);
//        portal = new VisionPortal.Builder()
//                .setCamera(BuiltinCameraDirection.BACK)
//                .addProcessor(aprilTagProcessor)
//                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, OpmodeConstants.WebcamName))
                .addProcessor(aprilTagProcessor)
                .build();
        portal.setProcessorEnabled(aprilTagProcessor, true);


        canlaunch = CanLaunch.ERROR;
        double voltage = floodgate.getVoltage();
        double amps = (voltage / 3.3) * 40.0;
        telemetry.addData("Current (Amps)", "%.2f A", amps);
        
    }
    @Override
    public void start(){
        follower.startTeleopDrive(true  );
        breakModeActive = true;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        pinpoint.update();
        follower.update();
        Drawing.drawDebug(follower);
        // Cache pinpoint values immediately after update to avoid redundant I2C reads
        cachedPosX = pinpoint.getPosX(DistanceUnit.MM);
        cachedPosY = pinpoint.getPosY(DistanceUnit.MM);
        cachedHeading = pinpoint.getHeading(AngleUnit.DEGREES);

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
            targetAngle += (gamepad1.x ? 0.013888 : 0.002777);// MAPPING
        }
        if (gamepad1.dpadLeftWasPressed()) {// MAPPING
            targetAngle -= (gamepad1.x ? 0.013888 : 0.002777);// MAPPING
        }
        double SERVO_MAXIMUM_POSITION = 0.25;
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

                case OFF:
                    selectedPreset = Preset.GOAL;
                    targetSpeed = goalSpeed;
                    targetAngle = goalAngle;
                    break;

                case GOAL:
                    selectedPreset = Preset.MIDDLE;
                    targetSpeed = midSpeed;
                    targetAngle = midAngle;
                    break;

                case MIDDLE:
                    selectedPreset = Preset.BACK;
                    targetSpeed = backlineSpeed;
                    targetAngle = backlineAngle;
                    break;

                case BACK:
                    selectedPreset = Preset.OFF;
                    targetSpeed = 0;
                    targetAngle = 0.25;
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
        boolean velocityInRange = (cachedLauncherVelocity >= targetSpeed - 40) && (cachedLauncherVelocity <= targetSpeed + 40);
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

        // Only enable AprilTag processing when alignment is requested (saves CPU/power)
        boolean alignmentRequested = gamepad1.right_trigger >= 0.2;
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

        // Reset X_MOVE when no target detected to prevent stale values causing drift
        double x_MOVE = 0;
        if (detection == null) {
            x_MOVE = 0;
            lastAlignmentError = 0;  // Reset derivative term
        }

        // Handle driving - only one Drive() call per loop
        boolean alignmentActive = false;

        if (alignmentRequested) {
            if (detection != null) {
                // Align to the target AprilTag when it is visible
                if (breakModeActive) {
                    breakModeActive = false;
                    follower.startTeleopDrive(false);
                }
                x_MOVE = calculateAlignmentCorrection(detection.ftcPose.z);
                follower.setTeleOpDrive(0, 0, -x_MOVE, true);
                alignmentActive = true;
                AutoMove = true;
            } else {
                // No tag: rotate toward the predefined aim heading using Pedro follower
                if (breakModeActive) {
                    breakModeActive = false;
                    follower.startTeleopDrive(false);
                    AutoMove = true;
                }

                double targetHeading = Math.toRadians();
                double headingError = targetHeading - follower.getHeading();

                // Normalize to [-π, π] for shortest rotation
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                double kP = 3.0;
                double exponentialFactor = 0.8;
                double normalizedError = Math.abs(headingError) / Math.PI;
                double exponentialGain = Math.pow(normalizedError, exponentialFactor);

                double rotationPower = kP * headingError * exponentialGain;
                rotationPower = Range.clip(rotationPower, -0.7, 0.7);
                follower.setTeleOpDrive(0, 0, rotationPower, true);
                alignmentActive = true;
            }
        }

        if (!alignmentActive) {
            if(!breakModeActive){
                breakModeActive = true;
                follower.startTeleopDrive(true);
            }
            AutoMove = false;
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        if (gamepad1.bWasPressed()) {
            follower.startTeleopDrive(true);
            follower.followPath(pathToBack.get());
            AutoMove = true;
        }
        if ((Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1)){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            AutoMove = false;
        }

        // Update light2 to show AprilTag alignment status (only when auto-aiming)
        if (alignmentRequested) {
            if (detection == null) {
                // Red: AprilTag not visible
                setLightColor(light2, StatusLightColor.RED);
            } else if (Math.abs(detection.ftcPose.z) <= ALIGNMENT_DEADBAND) {
                // Green: Aimed at goal within tolerance
                setLightColor(light2, StatusLightColor.GREEN);
            } else {
                // Yellow: AprilTag visible but not aligned
                setLightColor(light2, StatusLightColor.YELLOW);
            }
        } else {
            // Turn off light2 when not auto-aiming
            setLightColor(light2, StatusLightColor.OFF);
        }

        launcher.setVelocity(targetSpeed);

        // Only use manual drive if alignment is not active


        // Always add telemetry data and update for consistent display
        AddTelemetry();

        boolean rightBumper = gamepad1.right_bumper;
        now = System.currentTimeMillis();

        // Fire on initial press
        if (rightBumper && !lastRightBumper) {
            launchRequested = true;
            lastFireTime = now;
        }

        // Continuous fire when held (after interval)
        if (rightBumper && (now - lastFireTime > FIRE_INTERVAL) && launchState == LaunchState.IDLE) {
            launchRequested = true;

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
                cachedLauncherVelocity = launcher.getVelocity();
                if (cachedLauncherVelocity >= targetSpeed - 40 && cachedLauncherVelocity <= targetSpeed + 40) {
                    launchState = LaunchState.LAUNCH;
                    now = System.currentTimeMillis();
                    lastFireTime = now;
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
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                if (Current_speed == REV_SPEED) {
                    Current_speed = STOP_SPEED;
                    leftFeeder.setPower(Current_speed);
                    rightFeeder.setPower(Current_speed);
                }
                stoppy_servo.setPosition(0.3);
                break;

            case START_INTAKE:
                // Spin feeders backwards during the reverse window
                Current_speed = REV_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                // keep launcher servo in safe position
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                stoppy_servo.setPosition(0.55);
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
                stoppy_servo.setPosition(0.55);
                break;

            case SPIN:
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                IN_RPM = ((cachedIntakeVelocity / TPR_1620) * 60);
                IN_TARGET_RPM = ((SPIN_SPEED / 60.0) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                intake_ramp.setPosition(LAUNCH_POS);
                break;

            default:
                stoppy_servo.setPosition(0.3);
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

//        // Motor debug telemetry - verify BRAKE mode is working
//        telemetry.addLine("--- MOTOR DEBUG ---");
//        telemetry.addData("FL Power", "%.3f", frontLeftMotor.getPower());
//        telemetry.addData("FR Power", "%.3f", frontRightMotor.getPower());
//        telemetry.addData("BL Power", "%.3f", backLeftMotor.getPower());
//        telemetry.addData("BR Power", "%.3f", backRightMotor.getPower());
//        telemetry.addData("FL Mode", frontLeftMotor.getMode());
//        telemetry.addData("FL ZeroPower", frontLeftMotor.getZeroPowerBehavior());
//
        // AprilTag telemetry - show all detected tags for consistent display (prevents flickering)

        telemetry.update();
    }

    private void initialize_drive() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, OpmodeConstants.FrontLeftMotor);// DRIVE SETUP
        backLeftMotor = hardwareMap.get(DcMotorEx.class, OpmodeConstants.BackLeftMotor);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, OpmodeConstants.FrontRightMotor);
        backRightMotor = hardwareMap.get(DcMotorEx.class, OpmodeConstants.BackRightMotor);

        // Set run mode to RUN_WITHOUT_ENCODER for power control with reliable braking
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// DRIVE SETUP
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);// DRIVE SETUP
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);//
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMPORTANT: explicitly stop motors so BRAKE is engaged and telemetry reports BRAKE (not FLOAT)
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void initialize_launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, OpmodeConstants.LauncherName);
        //launcher motor
        double p = OpmodeConstants.Launcher_P;
        double i = OpmodeConstants.Launcher_I;
        double d = OpmodeConstants.Launcher_D;
        double f = OpmodeConstants.Launcher_F;
        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LEFT_LAUNCH_SERVO = hardwareMap.get(Servo.class, OpmodeConstants.AimServoName);
    }

    private void initialize_feeder() {
        leftFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.LeftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.RightFeederName);

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);//  DIRECTION SETUP
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initialize_intake() {
        intake = hardwareMap.get(DcMotorEx.class,  OpmodeConstants.IntakeName);
        intake_ramp = hardwareMap.get(Servo.class, OpmodeConstants.IntakeRampName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);// DIRECTION SETUP
    }

    private void initialize_pinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, OpmodeConstants.odometryName);
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
        if (Math.abs(forward) < 0.02) {forward = 0;}
        if (Math.abs(strafe) < 0.02) {strafe = 0;}
        if (Math.abs(rotate) < 0.02) {rotate = 0;}
        if (forward < -1) {forward = -1;}
        if (strafe < -1) {strafe = -1;}
        if (rotate < -1) {rotate = -1;}
        if (forward > 1) {forward = 1;}
        if (strafe > 1) {strafe = 1;}
        if (rotate > 1) {rotate = 1;}

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        // Slow mode when right stick button is pressed
        double speedMultiplier = gamepad1.right_stick_button ? 0.23 : 1.0;  // 100/435 ≈ 0.23

        // Calculate power for each wheel (mecanum drive kinematics)
        double flPower = ((forward - strafe - rotate) / denominator) * speedMultiplier;
        double frPower = ((forward + strafe + rotate) / denominator) * speedMultiplier;
        double blPower = ((forward + strafe - rotate) / denominator) * speedMultiplier;
        double brPower = ((forward - strafe + rotate) / denominator) * speedMultiplier;

        // Use setPower() consistently for reliable ZeroPowerBehavior.BRAKE support
        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }

    public AprilTagDetection getTargetDetection() {
        return targetDetection;
    }

    public void setTargetDetection(AprilTagDetection targetDetection) {
        this.targetDetection = targetDetection;
    }

    public abstract Pose set_goal_position();
    public abstract Pose set_backline_position();

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
        ERROR  // Error indicates RPM drop and motor bog(power set higher than expected with no reason), voltage criticaly low, and AMP draw over set limit, launch dissabled until AMPS drop below set [...]
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

    /**
     * Calculates the rotation correction needed to align with an AprilTag using PD control.
     * @param error The bearing error to the target (degrees)
     * @return The rotation power to apply (clipped to ALIGNMENT_MAX_POWER)
     */
    private double calculateAlignmentCorrection(double error) {
        // Apply deadband - if error is small enough, stop correcting
        if (Math.abs(error) < ALIGNMENT_DEADBAND) {
            lastAlignmentError = 0;
            filteredDerivative = 0;  // Reset filtered derivative when in deadband
            return 0;
        }

        // Calculate derivative (rate of change of error)
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastAlignmentTime) / 1000.0;  // Convert to seconds
        double rawDerivative = 0;
        if (dt > 0 && dt < 0.5) {  // Only use derivative if reasonable time delta
            rawDerivative = (error - lastAlignmentError) / dt;
        }

        // Apply low-pass filter to derivative to reduce noise-induced oscillation
        filteredDerivative = DERIVATIVE_FILTER_ALPHA * rawDerivative + (1 - DERIVATIVE_FILTER_ALPHA) * filteredDerivative;

        // PD control: output = P * error + D * derivative
        double pTerm = ALIGNMENT_P_GAIN * error;
        double dTerm = ALIGNMENT_D_GAIN * filteredDerivative;
        double correction = -(pTerm + dTerm);

        // Apply minimum power threshold to overcome static friction (prevents jerky start/stop)
        if (Math.abs(correction) > 0 && Math.abs(correction) < ALIGNMENT_MIN_POWER) {
            correction = Math.signum(correction) * ALIGNMENT_MIN_POWER;
        }

        // Clip to max power
        correction = Range.clip(correction, -ALIGNMENT_MAX_POWER, ALIGNMENT_MAX_POWER);

        // Store for next iteration
        lastAlignmentError = error;
        lastAlignmentTime = currentTime;

        return Range.clip(-0.01*error, -ALIGNMENT_MAX_POWER, ALIGNMENT_MAX_POWER);
    }
}
