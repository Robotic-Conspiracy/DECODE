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

import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;


@Configurable // Panels
public abstract class pickup12main extends OpMode {
    public final double INTAKE_POS = .84; // .87MAX
    int timesToShoot = 3;
    public int starting_pose_x;
    public int starting_pose_y;
    int timesShot = 0;
    abstract void set_color();
    public String color;
    final ElapsedTime feedTimer = new ElapsedTime();
    final ElapsedTime waitTimer = new ElapsedTime();
    boolean doneLaunching = false;
    private Servo LEFT_LAUNCH_SERVO = null;
    private final double STOP_SPEED = 0.0;
    private double Current_speed = STOP_SPEED;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    public static int targetSpeed = 2380;//launch motor speed
    private Servo intake_ramp = null;
    public static double targetAngle = 0.1444;
    public static int INTAKE_SPEED = 1600; //RPM

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int nextPathState;
    public Paths paths; // Paths defined in the Paths class
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
        set_starting_pose();
        follower.setStartingPose(new Pose(starting_pose_x,starting_pose_y,Math.toRadians(90)));

        // Create Paths object first (without building paths yet)
        paths = new Paths();

        // set_color() populates the path coordinates on the paths object
        set_color();

        // Now build the paths using the populated coordinates
        paths.buildPaths(follower);

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

    abstract void set_starting_pose();

    private boolean launch() {
        double FEED_TIME_SECONDS = 0.15;




        // If we've already shot enough for this launch call, do nothing
        if (timesShot >= timesToShoot) {
            return true;
        }

        double velocity = launcher.getVelocity();

        // Only start a feed cycle when launcher is up to speed and wait timer elapsed
        if ((velocity >= targetSpeed - 40 && velocity <= targetSpeed + 40) && waitTimer.seconds() > 0.5) {
            // Start feeding only if not already feeding
            double FULL_SPEED = 1.0;
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
        return timesShot == timesToShoot;
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

        public PathChain prePickup1;
        public PathChain pickup1;
        public PathChain shootPickup1;

        public PathChain prePickup2;
        public PathChain pickUp2;
        public PathChain shootPickup2;

        public PathChain prePickup3;
        public PathChain pickup3;
        public PathChain shootPickup3;

        public PathChain park;

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
        public Pose parkStart, parkEnd;
        public  int headShootPreload1;
        public  int headShootPreload2;
        public int headprePickup11;
        public int headprePickup12;
        public int headpickup1;
        public int headshootPickup11;
        public int headshootPickup12;
        public int headprePickup21;
        public int headprePickup22;
        public int headpickUp2;
        public int headshootPickup21;
        public int headshootPickup22;
        public int headprePickup31;
        public int headprePickup32;
        public int headpickup3;
        public int headshootPickup31;
        public int headshootPickup32;
        public int headpark;



        // No-arg constructor - fields will be populated by set_color() before buildPaths() is called
        public Paths() {
        }

        // Build all paths using the populated pose/heading fields
        public void buildPaths(Follower follower) {
            // shootPreload
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPreloadStart, shootPreloadEnd)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headShootPreload1),Math.toRadians(headShootPreload2))
                    .build();
            // prePickup1
            prePickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup1Start, prePickup1End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headprePickup11), Math.toRadians(headprePickup12))
                    .build();

            // pickup1

            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickup1Start, pickup1End)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(headpickup1))
                    .build();

            // shootPickup1

            shootPickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup1Start, shootPickup1End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headshootPickup11), Math.toRadians(headshootPickup12))
                    .build();



            // prePickup2

            prePickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup2Start, prePickup2End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headprePickup21), Math.toRadians(headprePickup22))
                    .build();

            // pickUp2

            pickUp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUp2Start, pickUp2End)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(headpickUp2))
                    .build();

            // shootPickup2

            shootPickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup2Start, shootPickup2End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headshootPickup21), Math.toRadians(headshootPickup22))
                    .build();



            // prePickup3

            prePickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(prePickup3Start, prePickup3End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headprePickup31), Math.toRadians(headprePickup32))
                    .build();

            // pickup3

            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickup3Start, pickup3End)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(headpickup3))
                    .build();

            // shootPickup3

            shootPickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPickup3Start, shootPickup3End)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headshootPickup31), Math.toRadians(headshootPickup32))

                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(parkStart, parkEnd)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(headpark))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        double TPR_1620 = 103.8;
        double LAUNCH_POS = 0.61;

        if(pathState == 3 || pathState == 6 || pathState == 9){
            follower.setMaxPower(0.35);
        } else {
            follower.setMaxPower(1);
        }
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
                    targetSpeed = 2380;
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
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);

                    tryFollowWithPoseRetry(paths.shootPickup1, paths.shootPickup1Start, "shootPickup1");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 5;
                    targetSpeed = 2380;
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
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);

                    tryFollowWithPoseRetry(paths.shootPickup2, paths.shootPickup2Start, "shootPickup2");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 100;
                    nextPathState = 8;
                    targetSpeed = 2480;
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
                    targetSpeed = 1900;
                    targetAngle = (double) 55 /360;
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
                    launcher.setVelocity(targetSpeed);
                    LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                    intake_ramp.setPosition(LAUNCH_POS);
                    intake.setVelocity(0);

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
                    panelsTelemetry.debug("Action", "Following park");
                    panelsTelemetry.debug("Path NonNull", paths != null && paths.park != null);
                    double dx = follower.getPose().getX() - paths.parkStart.getX();
                    double dy = follower.getPose().getY() - paths.parkStart.getY();
                    panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                    panelsTelemetry.update(telemetry);
                    targetSpeed = -20;
                    launcher.setVelocity(targetSpeed);

                    tryFollowWithPoseRetry(paths.park, paths.parkStart, "Path15");
                    waitingForPath = true;
                }
                if (waitingForPath && !follower.isBusy()) {
                    waitingForPath = false;
                    pathState = 0;
                    nextPathState = 0;
                }
                break;

            case 100:
                // TODO use case 100 for aiming to launch
                pathState = 101;
                break;
            case 101:
                launcher.setVelocity(targetSpeed);
                LEFT_LAUNCH_SERVO.setPosition(targetAngle);
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                doneLaunching = launch();
                if (doneLaunching){
                    pathState = 200;
                    timesShot = 0;
                }
                break;
            case 200:
                intake_ramp.setPosition(INTAKE_POS);
                double IN_TARGET_RPM = (((double) INTAKE_SPEED / 60) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                //LEFT_LAUNCH_SERVO.setPosition(0);
                pathState = nextPathState;
                break;
        }
    }
}
