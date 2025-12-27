package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import kotlin.NotImplementedError;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue 12 ball -- no sort")
public class BackOpenFieldAutoBlue extends OpMode {
    public static Follower follower;
    double LAUNCH_POS = 0.61;
    private ElapsedTime feedTimer = new ElapsedTime();
    private ElapsedTime waitTimer = new ElapsedTime();
    private int nextPathState = 0;
    private int timesShot = 0;
    double X_MOVE = 0;
    private final double TPR_435 = 384.5;
    double FL_RPM = 0;
    double FR_RPM = 0;
    double BL_RPM = 0;
    double BR_RPM = 0;
    double IN_RPM = 0;
    public final double INTAKE_POS = .84; // .87MAX
    public final int SPIN_SPEED = -500;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private double FL_MAX_RPM = 435;
    private double FR_MAX_RPM = 435;
    private double BL_MAX_RPM = 435;
    private double BR_MAX_RPM = 435;
    private DcMotorEx frontLeftMotor = null;
    private DcMotorEx backLeftMotor = null;
    private DcMotorEx frontRightMotor = null;
    private DcMotorEx backRightMotor = null;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 90;
    private final double TPR_1620 = 103.8;
    double IN_TARGET_RPM = 0;
    int timesToShoot = 4;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;private final double P = 203;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private Servo LEFT_LAUNCH_SERVO = null;
    public static int targetSpeed = 1680;//launch motor speed
    private Servo intake_ramp = null;
    public static double targetAngle = 90-38;
    public static int INTAKE_SPEED = 900;
    private final double I = 1.001;
    private final double D = 0.0015;
    private final double F = 0.1;
    private ElapsedTime Timer = new ElapsedTime();
    private ElapsedTime Timer2 = new ElapsedTime();
    private ElapsedTime Timer3 = new ElapsedTime();

    private solo_op_MAIN.IntakeState intakeState = null;

    private solo_op_MAIN.CanLaunch canlaunch = null;
    AnalogInput floodgate;
    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;

    private AprilTagDetection targetDetection = null;

    private Servo light1 = null;
    private Servo light2 = null;






    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // stabilization timer used after finishing the rotate-to-intake path
    private ElapsedTime stabilizeTimer = new ElapsedTime();
    private boolean stabilizeStarted = false;
    private final double STABILIZE_SECONDS = 0.3; // pause after rotation to let robot settle

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePoseMid = new Pose(60, 83, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseBack = new Pose(56, 12, Math.toRadians(113));
    private final Pose pickup1Pose = new Pose(24, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 84, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    // new helper paths for approaching intake after launch
    private PathChain advanceToPrePickup, prePickupToIntake;
    // pre-pickup pose and offset (in inches) to give margin for error
    private Pose prePickupPose;
    private final double PRE_PICKUP_OFFSET = 30.0; // inches margin
    private double Current_speed = STOP_SPEED;
    // flag to indicate an in-progress launch cycle (shots in progress)
    private boolean launchingNow = false;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePoseBack));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePoseBack.getHeading());

        // compute a pre-pickup pose: same Y as pickup1, X offset PRE_PICKUP_OFFSET inches toward the robot's side
        prePickupPose = new Pose(pickup1Pose.getX() + PRE_PICKUP_OFFSET, pickup1Pose.getY(), scorePoseBack.getHeading());

        // Path: advance from scoring pose to a position 10" to the side of the pickup and aligned in Y
        advanceToPrePickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBack, prePickupPose))
                .setLinearHeadingInterpolation(scorePoseBack.getHeading(), prePickupPose.getHeading())
                .build();

        // Path: from the pre-pickup pose rotate/drive to the pickup pose while facing 180 degrees (intake heading)
        prePickupToIntake = follower.pathBuilder()
                .addPath(new BezierLine(prePickupPose, pickup1Pose))
                .setLinearHeadingInterpolation(prePickupPose.getHeading(), Math.toRadians(180.0))
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBack, pickup1Pose))
                .setLinearHeadingInterpolation(scorePoseBack.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePoseBack))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePoseBack.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBack, pickup2Pose))
                .setLinearHeadingInterpolation(scorePoseBack.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePoseBack))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePoseBack.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseBack, pickup3Pose))
                .setLinearHeadingInterpolation(scorePoseBack.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePoseBack))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePoseBack.getHeading())
                .build();
    }

//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                timesShot = 0;
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    // arrived at scorePreload -> fire
//                    launch();
//                    follower.followPath(grabPickup1, true);
//                    setPathState(2);
//                    timesShot = 0;
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup1, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    // arrived at scorePickup1 -> fire
//                    launch();
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
//                    timesShot = 0;
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    // arrived at scorePickup2 -> fire
//                    launch();
//                    follower.followPath(grabPickup3, true);
//                    setPathState(6);
//                    timesShot = 0;
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup3, true);
//                    // arrived at scorePickup3 -> fire
//                    launch();
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    setPathState(-1);
//                    timesShot = 0;
//                }
//                break;
//        }
//    }
    public void auton_path_update(){
        switch (pathState) {
        case 1:
            follower.followPath(scorePreload,true);
            pathState = 2;
            // after scoring/aiming+launch we will first advance to a pre-pickup approach sequence
            nextPathState = 11;
            break;
            case 11:
                // drive forward from scoring position to the pre-pickup coordinate (y aligned, x offset by PRE_PICKUP_OFFSET)
                if (!follower.isBusy()) {
                    // telemetry so you can confirm the computed pre-pickup position
                    telemetry.addData("prePickupX", prePickupPose.getX());
                    telemetry.addData("prePickupY", prePickupPose.getY());
                    telemetry.addData("prePickupHeadingDeg", Math.toDegrees(prePickupPose.getHeading()));
                    telemetry.update();
                    follower.followPath(advanceToPrePickup, true);
                    pathState = 12;
                }
                break;
            case 12:
                // once at pre-pickup pose start the rotate/drive-to-intake path that ends facing 180°
                if (!follower.isBusy()) {
                    follower.followPath(prePickupToIntake, true);
                    // move to a short wait state so we don't immediately enter the intake logic
                    pathState = 13; // wait for this path to complete
                    // ensure stabilization flag is cleared when starting the path
                    stabilizeStarted = false;
                    // preserve where to go after intake
                    nextPathState = 6; // keep existing sequence behaviour
                }
                break;

            case 13:
                // wait for the rotate/drive-to-intake path to finish, then enter the intake state
                if (!follower.isBusy()) {
                    // when the follower reports finished, start stabilize timer once
                    if (!stabilizeStarted) {
                        stabilizeTimer.reset();
                        stabilizeStarted = true;
                        double headingDeg = Math.toDegrees(follower.getPose().getHeading());
                        telemetry.addData("post-rot heading deg", headingDeg);
                        telemetry.addData("stabilize", "starting");
                        telemetry.update();
                    } else {
                        // wait until stabilization time has elapsed
                        if (stabilizeTimer.seconds() >= STABILIZE_SECONDS) {
                            stabilizeStarted = false;
                            telemetry.addData("stabilize", "done");
                            telemetry.update();
                            // now that we've completed the path and paused, go to intake
                            pathState = 4;
                        } else {
                            telemetry.addData("stabilize_time", stabilizeTimer.seconds());
                            telemetry.update();
                        }
                    }
                }
                break;
            case 2:
                intake_ramp.setPosition(LAUNCH_POS);
                intake.setVelocity(0);
                LEFT_LAUNCH_SERVO.setPosition(.2);

                // get the latest april tag detection (choose the relevant id if needed)
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                AprilTagDetection detection = null;
                for (AprilTagDetection d : detections) {
                    if (d.id == 20) {
                        detection = d;
                        break;
                    }
                }

                // compute angle offset from the detection (use yaw from the detection.ftcPose which is in degrees)
                double angleOffsetRad = 0.0;
                double yawDeg = 0.0;
                if (detection != null) {
                    // use yaw (degrees) to compute heading offset
                    yawDeg = detection.ftcPose.yaw;
                    angleOffsetRad = Math.toRadians(yawDeg);
                }

                // build a temporary PathChain from current robot pose to the scoring pose with adjusted final heading
                Pose current = follower.getPose();
                Pose target = scorePoseBack;

                PathChain dynamicScore = follower.pathBuilder()
                        .addPath(new BezierLine(current, target))
                        .setLinearHeadingInterpolation(current.getHeading(), target.getHeading() + angleOffsetRad)
                        .build();

                // start the aiming path only if the follower isn't already following something
                if (!follower.isBusy()) {
                    follower.followPath(dynamicScore, true);
                }

                // go to wait/verify state so we can check alignment before launching
                pathState = 3;

                break;
            case 3:
                // wait for the aiming path to complete; when finished, check tag alignment and either launch or re-aim
                if(!follower.isBusy()) {
                    // get newest detection
                    List<AprilTagDetection> detections2 = aprilTagProcessor.getDetections();
                    AprilTagDetection detection2 = null;
                    for (AprilTagDetection d : detections2) {
                        if (d.id == 20) {
                            detection2 = d;
                            break;
                        }
                    }

                    boolean aligned = false;

                    if (detection2 != null) {
                        double yawDeg2 = detection2.ftcPose.yaw; // degrees
                        telemetry.addData("tag yaw deg", yawDeg2);

                        // if tag is within 5 degrees of center, consider aligned
                        if (Math.abs(yawDeg2) <= 5.0) {
                            aligned = true;
                        } else {
                            // re-aim: build a short path to correct heading and follow it
                            double reaimOffset = Math.toRadians(yawDeg2);
                            Pose cur = follower.getPose();
                            Pose tgt = scorePoseBack;
                            PathChain reaimPath = follower.pathBuilder()
                                    .addPath(new BezierLine(cur, tgt))
                                    .setLinearHeadingInterpolation(cur.getHeading(), tgt.getHeading() + reaimOffset)
                                    .build();
                            follower.followPath(reaimPath, true);

                            // stay in state 3 to wait for the re-aim to finish
                            pathState = 3;
                        }
                    } else {
                        // if no tag found, proceed (optional: you can choose to retry instead)
                        telemetry.addData("tag", "not found - launching anyway");
                        aligned = true;
                    }

                    if (aligned) {
                        // start the launch sequence only once
                        if (!launchingNow) {
                            launchingNow = true;
                            timesShot = 0;            // reset shot counter for this cycle
                            feedTimer.reset();       // ensure feed timing starts fresh
                            waitTimer.reset();       // ensure the wait timer enforces the delay before first shot
                            telemetry.addData("launching", "start");
                        }

                        // call launch() repeatedly until it increments timesShot to timesToShoot
                        launch();

                        // once finished, clear the flag and advance
                        if (timesShot >= timesToShoot) {
                            launchingNow = false;
                            telemetry.addData("launching", "done");
                            pathState = nextPathState;
                        }
                    }
                }
                break;
            case 4://intake
                intake_ramp.setPosition(INTAKE_POS);
                IN_RPM = ((intake.getVelocity() / TPR_1620) * 60);
                IN_TARGET_RPM = ((INTAKE_SPEED / 60) * TPR_1620);
                intake.setVelocity(IN_TARGET_RPM);
                LEFT_LAUNCH_SERVO.setPosition(.2);
                //LEFT_LAUNCH_SERVO.setPosition(0);
                pathState = nextPathState;
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    pathState = 4;
                    nextPathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    pathState = 2;
                    nextPathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    pathState = 4;
                    nextPathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    pathState = 2;
                    nextPathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    pathState = 4;
                    nextPathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    pathState = 2;
                    nextPathState = 0;
                }
                break;
                //grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;





    }}

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    private void launch() {
        double FEED_TIME_SECONDS = 0.15;

        // If we've already shot enough for this launch call, do nothing
        if (timesShot >= timesToShoot) {
            return;
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

    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) 16, TimeUnit.MILLISECONDS);
        }


        launcher.setVelocity(targetSpeed);
                // These loop the movements of the robot, these must be called continuously in order to work
                follower.update();

                auton_path_update();


                double velocity = launcher.getVelocity();
                // Feedback to Driver Hub for debugging
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.addData("target velocity", targetSpeed);
                telemetry.addData("velocity", velocity);
                telemetry.update();

        }
    /** This method is called once at the init of the OpMode. **/
@Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        initialize_feeder();
        initialize_launcher();
        initialize_intake();
        initialize_drive();
        LEFT_LAUNCH_SERVO.setPosition(.2);
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



        canlaunch = solo_op_MAIN.CanLaunch.ERROR;

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 1;
    }
    private void initialize_launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setVelocityPIDFCoefficients(P, I, D, F);
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
        rotate = Math.pow(rotate, 3);
        if (gamepad1.right_stick_button) {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 100;
        } else {
            FL_MAX_RPM = BL_MAX_RPM = FR_MAX_RPM = BR_MAX_RPM = 435;

        }
        double TPS_FL = frontLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_BL = backLeftMotor.getVelocity(); // default is ticks/sec
        double TPS_FR = frontRightMotor.getVelocity(); // default is ticks/sec
        double TPS_BR = backRightMotor.getVelocity(); // default is ticks/sec

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


    }}
