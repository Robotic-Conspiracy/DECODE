package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 * Limelight-Enabled version of Back3Main.
 * Adds pixel-perfect alignment in case 100 before firing.
 */
@Configurable
public abstract class LL_Back3Main extends OpMode {

    public final double INTAKE_POS = .84;
    int timesToShoot = 3;
    public int starting_pose_x;
    public int starting_pose_y;
    public int starting_pose_heading;
    int timesShot = 0;

    abstract void set_color();
    abstract void set_starting_pose();

    public String color;
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
    public static int targetSpeed = OpmodeConstants.AutoBacklineSpeed;
    private Servo intake_ramp = null;
    public static double targetAngle = OpmodeConstants.AutoBacklineAngle;
    public static int INTAKE_SPEED = 1600;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private int nextPathState;
    public Paths paths;
    private boolean waitingForPath = false;

    // Limelight
    protected Limelight3a limelight;
    protected double targetTxOffset = 0; // Adjustable offset for left/right aim
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

    private void initialize_feeder() {
        leftFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.LeftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, OpmodeConstants.RightFeederName);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initialize_intake() {
        intake = hardwareMap.get(DcMotorEx.class, OpmodeConstants.IntakeName);
        intake_ramp = hardwareMap.get(Servo.class, OpmodeConstants.IntakeRampName);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        initialize_launcher();
        initialize_intake();
        initialize_feeder();
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3a.class, OpmodeConstants.LimelightName);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        set_starting_pose();
        follower.setStartingPose(new Pose(starting_pose_x, starting_pose_y, Math.toRadians(starting_pose_heading)));

        paths = new Paths(follower);
        set_color();
        paths.buildPaths(follower);

        pathState = 1;
        nextPathState = 2;
        waitingForPath = false;

        panelsTelemetry.debug("Status", "LL_Back3Main Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    private boolean launch() {
        double FEED_TIME_SECONDS = 0.15;
        if (timesShot >= timesToShoot) return true;

        double velocity = launcher.getVelocity();
        if ((velocity >= targetSpeed - 40 && velocity <= targetSpeed + 40) && waitTimer.seconds() > 0.5) {
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
        autonomousPathUpdate();
        follower.update();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("LL TX", (limelight != null && limelight.getLatestResult() != null) ? limelight.getLatestResult().getTx() : "N/A");
        panelsTelemetry.update(telemetry);
    }

    /**
     * MAIN AUTO LOOP
     * This is a "State Machine." It follows a sequence of steps (cases).
     * 1. Follow Path to Goal
     * 100. Limelight "Micro-Adjustment" (Decoupled from Pathing)
     * 101. Launch Pixels
     * 2. Follow Path to Park
     */
    public void autonomousPathUpdate() {
        double LAUNCH_POS = 0.61;

        switch (pathState) {
            case 1:
                // --- MACRO MOVEMENT ---
                // We use Pedro Pathing to move across the field.
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.shootPreload, paths.shootPreloadStart, "shootPreload");
                    waitingForPath = true;
                }
                // When Pedro says "I'm done moving," we hand control to the Limelight.
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100; // Hand off to Limelight Alignment
                    nextPathState = 2; // After shooting, we will go to case 2
                    firstTimeInState100 = true;
                }
                break;

            case 100: 
                // --- MICRO ADJUSTMENT (DECOUPLING) ---
                // "Decoupling" means we stop the Path Follower's autonomous brain 
                // and use its raw motor powers for our own Vision Loop.
                if (firstTimeInState100) {
                    limelightLockTimer.reset();
                    firstTimeInState100 = false;
                }

                LLResult result = limelight.getLatestResult();
                
                // FAIL-SAFES: 
                // 1. TIMEOUT: If vision is blocked or laggy, don't wait forever.
                // 2. NO TARGET: If the camera is blind, just shoot anyway using the path's heading.
                // 3. LOCKED: If we are within 0.5 degrees, we are done!
                boolean timeout = limelightLockTimer.seconds() > OpmodeConstants.LIMELIGHT_AUTO_LOCK_TIMEOUT;
                boolean noTarget = (result == null || !result.isValid());
                
                double error = noTarget ? 0 : result.getTx() - targetTxOffset;
                boolean locked = Math.abs(error) < OpmodeConstants.LIMELIGHT_AUTO_LOCK_THRESHOLD;

                if (noTarget || timeout || locked) {
                    // STOP the Vision Loop and move to the Shooting state.
                    follower.setTeleOpMovement(0, 0, 0); 
                    pathState = 101; 
                } else {
                    // P-LOOP: The further off we are (error), the faster we turn.
                    double turnPower = error * OpmodeConstants.Limelight_P;
                    turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
                    
                    // Here we use setTeleOpMovement. This is "Decoupling."
                    // We tell Pedro: "I'm driving now, just give me turn power!"
                    follower.setTeleOpMovement(0, 0, turnPower);
                }
                break;

            case 101: 
                // --- ACTION STATE ---
                // The robot is now perfectly aimed. Fire the launcher.
                launcher.setVelocity(targetSpeed);
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                doneLaunching = launch();
                if (doneLaunching){
                    timesShot = 0;
                    pathState = 200;
                    waitTimer.reset();
                }
                break;

            case 2:
                // --- BACK TO MACRO ---
                // Once shooting is done, we resume Pedro Pathing to travel to the next spot.
                if (!waitingForPath && !follower.isBusy()) {
                    tryFollowWithPoseRetry(paths.park, paths.parkStart, "park");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 0; // End of Auto
                }
                StaticCommunism.pose = follower.getPose();
                break;

            case 200:
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
        public PathChain shootPreload, park;
        public Pose shootPreloadStart, shootPreloadEnd, parkStart, parkEnd;
        public int headShootPreload1, headShootPreload2, headPark1, headPark2;
        public Paths(Follower f) {}
        public void buildPaths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(new BezierLine(shootPreloadStart, shootPreloadEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(headShootPreload1), Math.toRadians(headShootPreload2)).build();
            park = follower.pathBuilder().addPath(new BezierLine(parkStart, parkEnd))
                    .setLinearHeadingInterpolation(Math.toRadians(headPark1), Math.toRadians(headPark2)).build();
        }
    }
}
