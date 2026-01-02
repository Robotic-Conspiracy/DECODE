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

import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;

    @Autonomous(name = "donrt run ", group = "Autonomous")
    @Configurable // Panels
    @SuppressWarnings("unused")
    public abstract class back3main extends OpMode {

        public final double INTAKE_POS = .84; // .87MAX
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

        @Override
        public void init() {

            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            initialize_launcher();
            initialize_intake();
            initialize_feeder();
            follower = Constants.createFollower(hardwareMap);
            // set starting pose to match the first path point (was 72,8) so the follower won't reject the path
            set_starting_pose();
            follower.setStartingPose(new Pose(starting_pose_x, starting_pose_y, Math.toRadians(starting_pose_heading)));

            // Create Paths object first (without building paths yet)
            paths = new Paths(follower);

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
            autonomousPathUpdate();
            follower.update(); // Update Pedro Pathing
            // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

        public static class Paths {

            public PathChain shootPreload;
            public double Wait2;
            public PathChain park;
            public Pose shootPreloadStart, shootPreloadEnd;
            public Pose parkStart, parkEnd;
            public int headShootPreload1;
            public int headShootPreload2;
            public int headPark1;
            public int headPark2;


            public Paths(Follower follower) {
                // constructor intentionally left simple; set_color() will populate Pose/heading fields
            }

            public void buildPaths(Follower follower){
                // Build shootPreload path using the Pose values provided by set_color()
                // Fallback to safe defaults if the Poses are not set
                shootPreload = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(shootPreloadStart, shootPreloadEnd)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(headShootPreload1), Math.toRadians(headShootPreload2))
                        .build();
                // Build park path
                park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(parkStart, parkEnd)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(headPark1), Math.toRadians(headPark2))
                        .build();
            }
        }


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

        public void autonomousPathUpdate() {
            double TPR_1620 = 103.8;
            double LAUNCH_POS = 0.61;

            //if (pathState == 3 || pathState == 6 || pathState == 9) {
                //follower.setMaxPower(0.5);
            //} else {
               // follower.setMaxPower(1);
            //}
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
                        panelsTelemetry.debug("Path NonNull", paths != null && paths.park != null);
                        //double dx = follower.getPose().getX() - paths.park.getX();
                        //double dy = follower.getPose().getY() - paths.park.getY();
                        //panelsTelemetry.debug("Dist to start", Math.hypot(dx, dy));
                        panelsTelemetry.update(telemetry);

                        tryFollowWithPoseRetry(paths.park,paths.parkStart, "prePickup1");
                        waitingForPath = true;
                    }
                    if (waitingForPath && !follower.isBusy()) {
                        waitingForPath = false;
                        pathState = 0;
                        nextPathState = 3;
                    }
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
                        pathState = nextPathState;
                        timesShot = 0;
                    }
                    break;


            }
        }
    }
