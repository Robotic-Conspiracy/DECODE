package org.firstinspires.ftc.teamcode.soloOP;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.autos.pedroPathing.Tuning.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Configurable
public abstract class aimingTest extends OpMode {

    //wheels
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    //shooter
    private DcMotorEx launcher;
    private Servo launchAngleServo;

    //intake
    private DcMotorEx intake;
    private Servo intakeRamp;

    //lights
    private Servo canShootLight;
    private Servo presetLight;

    //camera
    private VisionPortal camera;
    private AprilTagProcessor aprilTagProcessor;


    //pathing
    public Follower follower;
    public PathChain path;
    public Pose pedroPose;
    public Pose goalPosition;


    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        init_wheels();
        init_pedro();
        goalPosition = set_goal_position();
        Drawing.init();

    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();



        Drawing.drawDebug(follower);

        //drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if(gamepad1.b){
//            double heading = follower.getHeading();
//            double angle = Math.atan((goalPosition.getY() - follower.getPose().getY()) / (goalPosition.getX()-follower.getPose().getX()));
//            double angle_to_target = heading-angle;
//            drive(0, 0, Range.clip(0.5*(angle_to_target/angle), -0.5, 0.5));
            double heading = follower.getHeading();
            double angle = Math.atan2((goalPosition.getY() - follower.getPose().getY()), (goalPosition.getX() - follower.getPose().getX()));

            // Calculate the shortest angular distance (flipped)
            double angle_to_target = heading - angle;


            // Normalize to [-π, π] to ensure shortest rotation path
            while (angle_to_target > Math.PI) angle_to_target -= 2 * Math.PI;
            while (angle_to_target < -Math.PI) angle_to_target += 2 * Math.PI;

            // Exponential control: speed decreases exponentially as angle approaches zero
            double kP = 3; // Base proportional gain
            double exponentialFactor = 1; // Controls steepness of exponential curve
            double normalizedError = Math.abs(angle_to_target) / Math.PI; // Normalize to [0, 1]
            double exponentialGain = Math.pow(normalizedError, exponentialFactor);


            double rotationPower = kP * angle_to_target;
            rotationPower = Range.clip(rotationPower, -1.0, 1.0);

            panelsTelemetry.addData("heading", Math.toDegrees(follower.getHeading()));
            panelsTelemetry.addData("rotation power", rotationPower);
            panelsTelemetry.addData("target heading", Math.toDegrees(angle));
            panelsTelemetry.update();
            //drive(0, 0, rotationPower);
            follower.setTeleOpDrive(0,0, rotationPower, true);
        } else {
            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
        }


    }

    public void init_wheels(){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");// DRIVE SETUP
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");

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

    public void init_shooter(){

    }

    public void init_camera(){

    }

    public void init_lights(){

    }

    public void init_intake(){

    }

    public void init_pedro(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 9.6, Math.toRadians(90)));
    }

    public void drive(double forward, double strafe, double rotate){
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

    public void aim(){

    }

    public void handle_servo(){

    }

    public void handle_intake(){

    }

    public void handle_shooting(){

    }

    public PathChain generate_pathing_rotation(){
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), new Pose(follower.getPose().getX()+0.0001, follower.getPose().getX()+0.0001))
                )
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.atan(
                        (goalPosition.getY() - follower.getPose().getY()) / (goalPosition.getX()-follower.getPose().getX())
                        )
                ).build();
    }
    public void handle_pedro(){
        follower.update();

        if(path == null){
            path = generate_pathing_rotation();
        }

        if(gamepad1.b){
            follower.resumePathFollowing();
            follower.followPath(path);
            path = generate_pathing_rotation();
        } else {
            follower.pausePathFollowing();
        }
    }

    public abstract Pose set_goal_position();

    private enum LAUNCH_STATE{
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private enum INTAKE_STATE{
        SPIN,
        INTAKE,
        READY,
        NOT_READY,
        START_INTAKE
    }

    private enum PRESET{
        CUSTOM,
        GOAL,
        MIDDLE,
        JUGGLE,
        BACK,
        OFF
    }

    private enum CAMERA_STATE{
        ON,
        OFF,
        ERROR
    }

}
