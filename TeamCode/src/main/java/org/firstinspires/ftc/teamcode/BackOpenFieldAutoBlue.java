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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue 12 ball -- no sort")
public class BackOpenFieldAutoBlue extends OpMode {
    public static Follower follower;
    private ElapsedTime feedTimer = new ElapsedTime();
    private ElapsedTime waitTimer = new ElapsedTime();
    private int timesShot = 0;
    public final double INTAKE_POS = .84; // .87MAX
    public final int SPIN_SPEED = -500;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double SERVO_MINIMUM_POSITION = 0;
    private final double SERVO_MAXIMUM_POSITION = 90;
    private final double TPR_1620 = 103.8;
    double IN_TARGET_RPM = 0;
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
    private LaunchState launchState = null;
    private solo_op_MAIN.IntakeState intakeState = null;
    private Preset selectedPreset = null;
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

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePoseMid = new Pose(60, 83, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseBack = new Pose(56, 12, Math.toRadians(113));
    private final Pose pickup1Pose = new Pose(24, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 84, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private double Current_speed = STOP_SPEED;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePoseBack));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePoseBack.getHeading());

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                            follower.followPath(grabPickup1, true);
                            setPathState(2);
                        }

                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    launch();
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private void launch() {
        //Launch servo objects and vars
        double FEED_TIME_SECONDS = 0.15;

        double velocity = launcher.getVelocity();
        if(velocity >= targetSpeed -200 && velocity <= targetSpeed +200){
            Current_speed = FULL_SPEED;
            leftFeeder.setPower(Current_speed);
            rightFeeder.setPower(Current_speed);
            Timer.reset();
            if(Timer.seconds() > FEED_TIME_SECONDS){
                launchState = LaunchState.IDLE;
                Current_speed = STOP_SPEED;
                leftFeeder.setPower(Current_speed);
                rightFeeder.setPower(Current_speed);

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
            selectedPreset = Preset.BACK;
            switch (selectedPreset) {
                case CUSTOM:

                    targetSpeed = 1500;
                    targetAngle = 90 - 17;
                    break;
                case BACK:

                    targetSpeed = 1980;
                    targetAngle = 90 - 37;
                    break;
                case MIDDLE:

                    targetSpeed = 2360;
                    targetAngle = 90 - 38;
                    break;
                case JUGGLE:

                    targetSpeed = 660;
                    targetAngle = 90 - 22;
                    break;
                case OFF:

                    targetSpeed = 0;
                    targetAngle = 90 - 0;
                    break;

                case GOAL:
                    targetSpeed = 1500;
                    targetAngle = 90 - 14;
                    break;
            }
        launcher.setVelocity(targetSpeed);
                // These loop the movements of the robot, these must be called continuously in order to work
                follower.update();
                timesShot = 0;
                if (timesShot <= 4) {
                    if (feedTimer.seconds() > 0.15) {
                        launch();
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        timesShot += 1;
                        feedTimer.reset();
                    }
                }if (timesShot <= 4) {
                    if (feedTimer.seconds() > 0.3) {
                        launch();
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        timesShot += 1;
                        feedTimer.reset();
                 }if (timesShot >= 4) {
                autonomousPathUpdate();
            }
            }
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
        setPathState(0);
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

}

