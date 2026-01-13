package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3a;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.StaticCommunism;
import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;

/**
 * Limelight-Enabled version of pickup12main.
 * Uses case 100 for non-blocking alignment before every sequence of shots.
 */
@Configurable
public abstract class LL_pickup12main extends OpMode {

    public final double INTAKE_POS = OpmodeConstants.IntakeRampIntakePos;
    int timesToShoot = 3;
    public int starting_pose_x;
    public int starting_pose_y;
    int timesShot = 0;
    
    abstract void set_color();
    abstract void set_starting_pose();
    
    public String color;
    private Servo stoppy_servo;
    final ElapsedTime feedTimer = new ElapsedTime();
    final ElapsedTime waitTimer = new ElapsedTime();
    final ElapsedTime limelightLockTimer = new ElapsedTime();
    
    boolean doneLaunching = false;
    private Servo LEFT_LAUNCH_SERVO = null;
    private final double STOP_SPEED = 0.0;
    private double Current_speed = STOP_SPEED;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private Servo intake_ramp = null;

    public static int INTAKE_SPEED = 1600;
    public static int backlineSpeed = OpmodeConstants.AutoBacklineSpeed;
    public static double backlineAngle = OpmodeConstants.AutoBacklineAngle;
    public static int midSpeed = OpmodeConstants.AutoMidSpeed;
    public static double midAngle = OpmodeConstants.AutoMidAngle;

    public int targetSpeed = backlineSpeed;
    public double targetAngle = backlineAngle;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private int nextPathState;
    public Paths paths;
    private boolean waitingForPath = false;
    private final long PATH_TIMEOUT_MS = 4000;
    private long currentPathStartTime = 0;
    private final double START_TOLERANCE_INCHES = 6.0;

    // Limelight
    protected Limelight3a limelight;
    protected double targetTxOffset = 0;
    private boolean firstTimeInState100 = true;

    private void initialize_launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, OpmodeConstants.LauncherName);
        double p = OpmodeConstants.Launcher_P;
        double i = OpmodeConstants.Launcher_I;
        double d = OpmodeConstants.Launcher_D;
        double f = OpmodeConstants.Launcher_F;
        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFT_LAUNCH_SERVO = hardwareMap.get(Servo.class, OpmodeConstants.AimServoName);
    }

    private void initialize_feeder(){
        leftFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.LeftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.RightFeederName);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initialize_intake(){
        intake = hardwareMap.get(DcMotorEx.class, OpmodeConstants.IntakeName);
        intake_ramp = hardwareMap.get(Servo.class, OpmodeConstants.IntakeRampName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init() {
        stoppy_servo = hardwareMap.get(Servo.class, OpmodeConstants.IntakeStopperName);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        initialize_launcher();
        initialize_intake();
        initialize_feeder();
        
        // Limelight
        limelight = hardwareMap.get(Limelight3a.class, OpmodeConstants.LimelightName);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        set_starting_pose();
        follower.setStartingPose(new Pose(starting_pose_x, starting_pose_y, Math.toRadians(90)));

        paths = new Paths();
        set_color();
        paths.buildPaths(follower);

        pathState = 1;
        nextPathState = 2;
        waitingForPath = false;
        targetSpeed = backlineSpeed;
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    private boolean launch() {
        double FEED_TIME_SECONDS = OpmodeConstants.FeedTimeSeconds;
        if (timesShot >= timesToShoot) return true;
        double velocity = launcher.getVelocity();
        if ((velocity >= targetSpeed - 40 && velocity <= targetSpeed + 40) && waitTimer.seconds() > 0.3) {
            if (Current_speed != 1.0) {
                Current_speed = 1.0;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                feedTimer.reset();
            }
            if (feedTimer.seconds() > FEED_TIME_SECONDS) {
                Current_speed = STOP_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                timesShot += 1;
                waitTimer.reset();
            }
        }
        return timesShot == timesToShoot;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }

    /**
     * MAIN AUTO LOOP
     * Logic Flow:
     * 1. Path to shooting spot (Macro)
     * 100. Limelight Align (Micro - Decoupled)
     * 101. Shoot (Action)
     * 2-10. Repeat for multiple pickups
     */
    public void autonomousPathUpdate() {
        double TPR_1620 = 103.8;
        double LAUNCH_POS = 0.61;

        if(pathState == 3 || pathState == 6 || pathState == 9){
            follower.setMaxPower(0.6);
            stoppy_servo.setPosition(0.55);
        } else {
            follower.setMaxPower(1);
            stoppy_servo.setPosition(0.3);
        }

        switch (pathState) {
            case 1:
                // Macro move to first spot
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.shootPreload, paths.shootPreloadStart, "shootPreload");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100; // Hand off to Limelight
                    nextPathState = 2; // Next path logic
                    firstTimeInState100 = true;
                }
                break;
            case 2:
                // Resume pathing to pickup
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.prePickup1, paths.prePickup1Start, "prePickup1");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 200;
                    nextPathState = 3;
                }
                break;
            case 3:
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.pickup1, paths.pickup1Start, "pickup1");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 4;
                    nextPathState = 4;
                }
                break;
            case 4:
                // Move toward shooting spot
                if (!waitingForPath && !follower.isBusy()) {
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);
                    tryFollowWithPoseRetry(paths.shootPickup1, paths.shootPickup1Start, "shootPickup1");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100; // Hand off to Limelight
                    nextPathState = 5;
                    firstTimeInState100 = true;
                }
                break;
            case 5:
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.prePickup2, paths.prePickup2Start, "prePickup2");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 200;
                    nextPathState = 6;
                }
                break;
            case 6:
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.pickUp2, paths.pickUp2Start, "pickUp2");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 7;
                    nextPathState = 7;
                }
                break;
            case 7:
                if (!waitingForPath && !follower.isBusy()) {
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);
                    tryFollowWithPoseRetry(paths.shootPickup2, paths.shootPickup2Start, "shootPickup2");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100; // Hand off to Limelight
                    nextPathState = 8;
                    firstTimeInState100 = true;
                }
                break;
            case 8:
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.prePickup3, paths.prePickup3Start, "prePickup3");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    targetSpeed = midSpeed;
                    targetAngle = midAngle;
                    pathState = 200;
                    nextPathState = 9;
                }
                break;
            case 9:
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.pickup3, paths.pickup3Start, "pickup3");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 10;
                    nextPathState = 10;
                }
                break;
            case 10:
                if (!waitingForPath && !follower.isBusy()) {
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);
                    tryFollowWithPoseRetry(paths.shootPickup3, paths.shootPickup3Start, "shootPickup3");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100; // Hand off to Limelight
                    nextPathState = 11;
                    firstTimeInState100 = true;
                }
                break;
            case 11:
                if (!waitingForPath && !follower.isBusy()) {
                    targetSpeed = -20;
                    launcher.setVelocity(targetSpeed);
                    tryFollowWithPoseRetry(paths.park, paths.parkStart, "park");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 0;
                }
                StaticCommunism.pose = follower.getPose();
                break;

            case 100: // --- DECOUPLED VISION ALIGNMENT ---
                if (firstTimeInState100) {
                    limelightLockTimer.reset();
                    firstTimeInState100 = false;
                }
                LLResult result = limelight.getLatestResult();
                boolean timeout = limelightLockTimer.seconds() > OpmodeConstants.LIMELIGHT_AUTO_LOCK_TIMEOUT;
                boolean noTarget = (result == null || !result.isValid());
                double error = noTarget ? 0 : result.getTx() - targetTxOffset;
                boolean locked = Math.abs(error) < OpmodeConstants.LIMELIGHT_AUTO_LOCK_THRESHOLD;

                if (noTarget || timeout || locked) {
                    follower.setTeleOpMovement(0, 0, 0); // Release control back to state machine
                    pathState = 101;
                } else {
                    // Turn only (Micro precision)
                    double turnPower = error * OpmodeConstants.Limelight_P;
                    turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
                    follower.setTeleOpMovement(0, 0, turnPower);
                }
                break;

            case 101:
                // --- SHOOTING ACTION ---
                launcher.setVelocity(targetSpeed);
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                doneLaunching = launch();
                if (doneLaunching) {
                    pathState = 200;
                    timesShot = 0;
                    waitTimer.reset();
                }
                break;
            case 200:
                intake_ramp.setPosition(INTAKE_POS);
                double IN_TARGET_RPM = (((double) INTAKE_SPEED / 60) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                if (waitTimer.seconds() >= 0.3) {
                    pathState = nextPathState;
                }
                break;
        }
    }

    private void tryFollowWithPoseRetry(PathChain path, Pose startPose, String pathName) {
        if (path == null) return;
        follower.followPath(path);
    }

    public static class Paths {
        public PathChain shootPreload, prePickup1, pickup1, shootPickup1, prePickup2, pickUp2, shootPickup2, prePickup3, pickup3, shootPickup3, park;
        public Pose shootPreloadStart, shootPreloadEnd, prePickup1Start, prePickup1End, pickup1Start, pickup1End, shootPickup1Start, shootPickup1End, prePickup2Start, prePickup2End, pickUp2Start, pickUp2End, shootPickup2Start, shootPickup2End, prePickup3Start, prePickup3End, pickup3Start, pickup3End, shootPickup3Start, shootPickup3End, parkStart, parkEnd;
        public int headShootPreload1, headShootPreload2, headprePickup11, headprePickup12, headpickup1, headshootPickup11, headshootPickup12, headprePickup21, headprePickup22, headpickUp2, headshootPickup21, headshootPickup22, headprePickup31, headprePickup32, headpickup3, headshootPickup31, headshootPickup32, headpark;
        public Paths() {}
        public void buildPaths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(new BezierLine(shootPreloadStart, shootPreloadEnd)).setLinearHeadingInterpolation(Math.toRadians(headShootPreload1),Math.toRadians(headShootPreload2)).build();
            prePickup1 = follower.pathBuilder().addPath(new BezierLine(prePickup1Start, prePickup1End)).setLinearHeadingInterpolation(Math.toRadians(headprePickup11), Math.toRadians(headprePickup12)).build();
            pickup1 = follower.pathBuilder().addPath(new BezierLine(pickup1Start, pickup1End)).setConstantHeadingInterpolation(Math.toRadians(headpickup1)).build();
            shootPickup1 = follower.pathBuilder().addPath(new BezierLine(shootPickup1Start, shootPickup1End)).setLinearHeadingInterpolation(Math.toRadians(headshootPickup11), Math.toRadians(headshootPickup12)).build();
            prePickup2 = follower.pathBuilder().addPath(new BezierLine(prePickup2Start, prePickup2End)).setLinearHeadingInterpolation(Math.toRadians(headprePickup21), Math.toRadians(headprePickup22)).build();
            pickUp2 = follower.pathBuilder().addPath(new BezierLine(pickUp2Start, pickUp2End)).setConstantHeadingInterpolation(Math.toRadians(headpickUp2)).build();
            shootPickup2 = follower.pathBuilder().addPath(new BezierLine(shootPickup2Start, shootPickup2End)).setLinearHeadingInterpolation(Math.toRadians(headshootPickup21), Math.toRadians(headshootPickup22)).build();
            prePickup3 = follower.pathBuilder().addPath(new BezierLine(prePickup3Start, prePickup3End)).setLinearHeadingInterpolation(Math.toRadians(headprePickup31), Math.toRadians(headprePickup32)).build();
            pickup3 = follower.pathBuilder().addPath(new BezierLine(pickup3Start, pickup3End)).setConstantHeadingInterpolation(Math.toRadians(headpickup3)).build();
            shootPickup3 = follower.pathBuilder().addPath(new BezierLine(shootPickup3Start, shootPickup3End)).setLinearHeadingInterpolation(Math.toRadians(headshootPickup31), Math.toRadians(headshootPickup32)).build();
            park = follower.pathBuilder().addPath(new BezierLine(parkStart, parkEnd)).setConstantHeadingInterpolation(Math.toRadians(headpark)).build();
        }
    }
}
