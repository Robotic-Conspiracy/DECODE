package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "the 12 bluest of blue balls of the holiest holly pantheon", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends OpMode {
    public final double INTAKE_POS = .84; // .87MAX
    int timesToShoot = 3;
    int timesShot = 0;
    final ElapsedTime feedTimer = new ElapsedTime();
    final ElapsedTime waitTimer = new ElapsedTime();
    boolean doneLaunching = false;
    public final int SPIN_SPEED = -500;
    private Servo LEFT_LAUNCH_SERVO = null;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private double Current_speed = STOP_SPEED;

    private double IN_TARGET_RPM = 0;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    public static int targetSpeed = 2440;//launch motor speed
    private Servo intake_ramp = null;
    public static double targetAngle = 0.1444;
    public static int INTAKE_SPEED = 900;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int nextPathState;
    private Paths paths; // Paths defined in the Paths class
    // flag to indicate we've issued a followPath and are waiting for it to finish
    private boolean waitingForPath = false;

    // timeout to prevent getting stuck on a single path (ms)
    private static final long PATH_TIMEOUT_MS = 8000;
    private long currentPathStartTime = 0;

    // If the robot is further than this from a path start, we will reseat pose and retry (inches)
    private static final double START_TOLERANCE_INCHES = 6.0;

    private void initialize_launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        double p = 203;
        double i = 1.001;
        double d = 0.0015;
        double f = 0.1;
        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
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

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        initialize_launcher();
        initialize_intake();
        initialize_feeder();
        follower = Constants.createFollower(hardwareMap);
        // set starting pose to match the first path point (was 72,8) so the follower won't reject the path
        follower.setStartingPose(new Pose(63, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        // start the autonomous state machine at step 1
        pathState = 1;
        nextPathState = 2;
        waitingForPath = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Follower Null?", follower == null);
        panelsTelemetry.debug("Paths Null?", paths == null);
        panelsTelemetry.debug("Starting Path State", pathState);
        panelsTelemetry.update(telemetry);
    }
    private boolean launch() {
        double FEED_TIME_SECONDS = 0.15;


        double LAUNCH_POS = 0.61;

        // If we've already shot enough for this launch call, do nothing
        if (timesShot >= timesToShoot) {
            return true;
        }

        double velocity = launcher.getVelocity();

        // Only start a feed cycle when launcher is up to speed and wait timer elapsed
        if ((velocity >= targetSpeed - 200 && velocity <= targetSpeed + 200) && waitTimer.seconds() > 0.5) {
            // Start feeding only if not already feeding
            if (Current_speed != FULL_SPEED) {
                Current_speed = FULL_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                feedTimer.reset(); // start timing the feed pulse once
            }

            // Stop feeding after FEED_TIME_SECONDS and count the shot
            if (feedTimer.seconds() > FEED_TIME_SECONDS) {
                Current_speed = STOP_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);
                timesShot += 1;
                waitTimer.reset(); // enforce delay before next shot cycle
            }
        }
        if (timesShot == timesToShoot) {
            return true;
        }
        if (timesShot < timesToShoot) {
            return false;
        }
        return false;
    }

    @Override
    public void loop() {
        try {
            if (follower == null) {
                panelsTelemetry.debug("ERROR", "Follower is null - check Constants.createFollower");
                panelsTelemetry.update(telemetry);
                return;
            }

            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine

        } catch (Exception e) {
            panelsTelemetry.debug("Exception during loop", e.getMessage());
            panelsTelemetry.update(telemetry);
            return;
        }
        launcher.setVelocity(targetSpeed);
        LEFT_LAUNCH_SERVO.setPosition(targetAngle);
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        // if waiting for a path, check timeout
        if (waitingForPath) {
            long elapsed = System.currentTimeMillis() - currentPathStartTime;
            panelsTelemetry.debug("Path Elapsed (ms)", elapsed);
            if (elapsed > PATH_TIMEOUT_MS) {
                panelsTelemetry.debug("Path Timeout", "Aborting stuck path");
                waitingForPath = false;
                // advance to next state to avoid lockup
                pathState = nextPathState;
            }
        }
        panelsTelemetry.update(telemetry);
    }

    // Helper: attempt to follow a path; if follower doesn't become busy and the robot is far from the path start,
    // reseat the follower pose to the path start and retry once. This helps when localization is off and prevents
    // the opmode from getting stuck.
    private void tryFollowWithPoseRetry(PathChain path, Pose startPose, String pathName) {
        if (path == null) {
            panelsTelemetry.debug("tryFollow", pathName + " path is null");
            panelsTelemetry.update(telemetry);
            return;
        }

        try {
            follower.followPath(path);
            currentPathStartTime = System.currentTimeMillis();
        } catch (Exception e) {
            panelsTelemetry.debug("followPath Exception", pathName + ": " + e.getMessage());
            panelsTelemetry.update(telemetry);
            return;
        }

        panelsTelemetry.debug("Follower Busy after followPath", follower.isBusy());

        // If follower didn't become busy, check distance to the path start and try reseating the pose if it's large
        if (!follower.isBusy() && startPose != null) {
            double dx = follower.getPose().getX() - startPose.getX();
            double dy = follower.getPose().getY() - startPose.getY();
            double dist = Math.hypot(dx, dy);
            panelsTelemetry.debug("Dist to start (post-follow)", dist);
            if (dist > START_TOLERANCE_INCHES) {
                panelsTelemetry.debug("StartTooFar", pathName + " start too far (" + dist + "in). Reseating pose and retrying.");
                // reseat the follower pose to the path start and retry once
                follower.setStartingPose(startPose);
                try {
                    follower.followPath(path);
                    currentPathStartTime = System.currentTimeMillis();
                } catch (Exception e) {
                    panelsTelemetry.debug("followPath Exception", "retry " + pathName + ": " + e.getMessage());
                    panelsTelemetry.update(telemetry);
                    return;
                }
                panelsTelemetry.debug("Retry Follower Busy", follower.isBusy());
            }
        }

        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain shootPreload;
        public double Wait3;
        public PathChain prePickup1;
        public PathChain pickup1;
        public PathChain shootPickup1;
        public double Wait7;
        public PathChain prePickup2;
        public PathChain pickUp2;
        public PathChain shootPickup2;
        public double Wait10;
        public PathChain prePickup3;
        public PathChain pickup3;
        public PathChain shootPickup3;
        public double Wait14;
        public PathChain Path15;

        // expose start/end poses for diagnostics
        public Pose shootPreloadStart, shootPreloadEnd;
        public Pose prePickup1Start, prePickup1End;
        public Pose pickup1Start, pickup1End;
        public Pose shootPickup1Start, shootPickup1End;
        public Pose prePickup2Start, prePickup2End;
        public Pose pickUp2Start, pickUp2End;
        public Pose shootPickup2Start, shootPickup2End;
        public Pose prePickup3Start, prePickup3End;
        public Pose pickup3Start, pickup3End;
        public Pose shootPickup3Start, shootPickup3End;
        public Pose Path15Start, Path15End;


        public Paths(Follower follower) {
            // shootPreload
            shootPreloadStart = new Pose(63.000, 8.000);
            shootPreloadEnd = new Pose(57.812, 15.882);
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPreloadStart, shootPreloadEnd)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            Wait3 = 1000;

            // prePickup1
            prePickup1Start = new Pose(57.812, 15.882);
            prePickup1End = new Pose(52.729, 35.576);
            prePickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup1Start, prePickup1End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            // pickup1
            pickup1Start = new Pose(52.729, 35.576);
            pickup1End = new Pose(16.729, 35.365);
            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickup1Start, pickup1End)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // shootPickup1
            shootPickup1Start = new Pose(16.729, 35.365);
            shootPickup1End = new Pose(58.024, 15.882);
            shootPickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup1Start, shootPickup1End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait7 = 1000;

            // prePickup2
            prePickup2Start = new Pose(58, 16);
            prePickup2End = new Pose(52.518, 59.929);
            prePickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup2Start, prePickup2End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            // pickUp2
            pickUp2Start = new Pose(52.518, 59.929);
            pickUp2End = new Pose(17.365, 59.718);
            pickUp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUp2Start, pickUp2End)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // shootPickup2
            shootPickup2Start = new Pose(17.365, 59.718);
            shootPickup2End = new Pose(58.024, 15.882);
            shootPickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup2Start, shootPickup2End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait10 = 1000;

            // prePickup3
            prePickup3Start = new Pose(58.024, 15.882);
            prePickup3End = new Pose(50.824, 84.071);
            prePickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup3Start, prePickup3End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            // pickup3
            pickup3Start = new Pose(50.824, 84.071);
            pickup3End = new Pose(17.365, 83.859);
            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickup3Start, pickup3End)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // shootPickup3
            shootPickup3Start = new Pose(17.365, 83.859);
            shootPickup3End = new Pose(59.294, 84.282);
            shootPickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup3Start, shootPickup3End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Wait14 = 1000;

            Path15Start = new Pose(59.506, 70.518);
            Path15End = new Pose(25.200, 70.941);
            Path15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Path15Start, Path15End)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                // start the path once and wait for it to complete
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following shootPreload");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.shootPreload != null);
                    // log distance to start
                    double dx = follower.getPose().getX() - paths.shootPreloadStart.getX();
                    double dy = follower.getPose().getY() - paths.shootPreloadStart.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.shootPreload, paths.shootPreloadStart, "shootPreload");
                    waitingForPath = true;
                }
                // when the follower has finished the path, advance
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 2;
                }
                break;
            case 2:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following prePickup1");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.prePickup1 != null);
                    double dx = follower.getPose().getX() - paths.prePickup1Start.getX();
                    double dy = follower.getPose().getY() - paths.prePickup1Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

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
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following pickup1");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.pickup1 != null);
                    double dx = follower.getPose().getX() - paths.pickup1Start.getX();
                    double dy = follower.getPose().getY() - paths.pickup1Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

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
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following shootPickup1");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.shootPickup1 != null);
                    double dx = follower.getPose().getX() - paths.shootPickup1Start.getX();
                    double dy = follower.getPose().getY() - paths.shootPickup1Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.shootPickup1, paths.shootPickup1Start, "shootPickup1");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 5;
                }
                break;
            case 5:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following prePickup2");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.prePickup2 != null);
                    double dx = follower.getPose().getX() - paths.prePickup2Start.getX();
                    double dy = follower.getPose().getY() - paths.prePickup2Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

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
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following pickUp2");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.pickUp2 != null);
                    double dx = follower.getPose().getX() - paths.pickUp2Start.getX();
                    double dy = follower.getPose().getY() - paths.pickUp2Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

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
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following shootPickup2");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.shootPickup2 != null);
                    double dx = follower.getPose().getX() - paths.shootPickup2Start.getX();
                    double dy = follower.getPose().getY() - paths.shootPickup2Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.shootPickup2, paths.shootPickup2Start, "shootPickup2");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 8;
                }
                break;
            case 8:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following prePickup3");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.prePickup3 != null);
                    double dx = follower.getPose().getX() - paths.prePickup3Start.getX();
                    double dy = follower.getPose().getY() - paths.prePickup3Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.prePickup3, paths.prePickup3Start, "prePickup3");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 200;
                    nextPathState = 9;
                }
                break;
            case 9:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following pickup3");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.pickup3 != null);
                    double dx = follower.getPose().getX() - paths.pickup3Start.getX();
                    double dy = follower.getPose().getY() - paths.pickup3Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

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
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following shootPickup3");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.shootPickup3 != null);
                    double dx = follower.getPose().getX() - paths.shootPickup3Start.getX();
                    double dy = follower.getPose().getY() - paths.shootPickup3Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.shootPickup3, paths.shootPickup3Start, "shootPickup3");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 11;
                }
                break;
            case 11:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following Path15");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.Path15 != null);
                    double dx = follower.getPose().getX() - paths.Path15Start.getX();
                    double dy = follower.getPose().getY() - paths.Path15Start.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.Path15, paths.Path15Start, "Path15");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 0;
                    nextPathState = 0;
                }
                break;
            case 12:
                if (!waitingForPath && follower != null && !follower.isBusy()) {
                    panelsTelemetry.debug("Action", "Following shootPreload (12)");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.shootPreload != null);
                    double dx = follower.getPose().getX() - paths.shootPreloadStart.getX();
                    double dy = follower.getPose().getY() - paths.shootPreloadStart.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);

                    tryFollowWithPoseRetry(paths.shootPreload, paths.shootPreloadStart, "shootPreload(12)");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 2;
                }
                break;
            case 13:
                break;
            case 14:
                break;
            case 15:
                break;
            case 100:
                // TODO use case 100 for aiming to launch
                pathState = 101;
                break;
            case 101:
                // TODO use case 101 for launch
                doneLaunching = launch();
                if (doneLaunching){
                pathState = nextPathState;
                timesShot = 0;
                }
                break;
            case 200:
                // TODO use case 200 as a pause to start the intake
                pathState = nextPathState;
                break;
        }
    }
}
