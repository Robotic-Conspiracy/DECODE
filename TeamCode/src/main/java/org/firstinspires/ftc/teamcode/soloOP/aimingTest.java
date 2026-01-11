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

import org.firstinspires.ftc.teamcode.OpmodeConstants;
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

    private boolean breakModeActive = false;

    // --- Auto-aiming configuration ---
    // Configurable via FTC Dashboard
    public static double goalHeightInches = 36.0;           // Height of the goal basket (inches)
    public static double launcherHeightInches = 12.0;       // Height of launcher above ground (inches)
    public static double gravity = 386.0;                    // Gravity in inches/sec^2 (32.2 ft/s^2)

    // Velocity control parameters (for encoder-based velocity control)
    public static double launcherTicksPerRev = 537.7;       // Encoder ticks per revolution (adjust for your motor)
    public static double launcherWheelDiameterInches = 4.0; // Diameter of launcher wheel (inches)
    public static double launchVelocityFactor = 0.8;        // Efficiency factor (0-1) for actual vs theoretical velocity

    // Servo angle mapping (tune these to your servo/mechanism)
    public static double servoAngleMin = 0.0;               // Servo position for minimum angle
    public static double servoAngleMax = 1.0;               // Servo position for maximum angle
    public static double launchAngleMinDegrees = 30.0;      // Minimum launch angle (degrees)
    public static double launchAngleMaxDegrees = 60.0;      // Maximum launch angle (degrees)

    // Velocity limits (ticks per second)
    public static double minLauncherVelocity = 500.0;       // Minimum velocity in ticks/sec
    public static double maxLauncherVelocity = 2800.0;      // Maximum velocity in ticks/sec (adjust for your motor)
    // ---------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        init_wheels();
        init_shooter();
        init_pedro();
        goalPosition = set_goal_position();
        Drawing.init();

    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        breakModeActive = true;
    }

    @Override
    public void loop() {
        follower.update();



        Drawing.drawDebug(follower);

        //drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if(gamepad1.b){
            if(breakModeActive){
                breakModeActive = false;
                follower.startTeleopDrive(false);
            }

            double heading = follower.getHeading();
            // Use atan2 for correct quadrant handling
            double angle = Math.atan2((goalPosition.getY() - follower.getPose().getY()), (goalPosition.getX() - follower.getPose().getX()));

            // Calculate the shortest angular distance - FLIP the sign here
            double angle_to_target = angle - heading;  // Changed from heading - angle

            // Normalize to [-π, π] to ensure shortest rotation path
            while (angle_to_target > Math.PI) angle_to_target -= 2 * Math.PI;
            while (angle_to_target < -Math.PI) angle_to_target += 2 * Math.PI;

            double kP = 3;//1.5
            double exponentialFactor = 0.8;//0.8
            double normalizedError = Math.abs(angle_to_target) / Math.PI;
            double exponentialGain = Math.pow(normalizedError, exponentialFactor);

            double rotationPower = kP * angle_to_target * exponentialGain;
            rotationPower = Range.clip(rotationPower, -0.7, 0.7);

            // Automatically adjust launcher angle and velocity for current distance
            aim();

            panelsTelemetry.addData("heading", Math.toDegrees(follower.getHeading()));
            panelsTelemetry.addData("rotation power", rotationPower);
            panelsTelemetry.addData("target heading", Math.toDegrees(angle));
            panelsTelemetry.addData("angle error", Math.toDegrees(angle_to_target));
            panelsTelemetry.update();

            follower.setTeleOpDrive(0, 0, rotationPower, true);
        } else {
            if(!breakModeActive){
                breakModeActive = true;
                follower.startTeleopDrive(true);
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
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
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launchAngleServo = hardwareMap.get(Servo.class, OpmodeConstants.AimServoName);

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setVelocity(0);
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
        // Calculate horizontal distance to goal
        double dx = goalPosition.getX() - follower.getPose().getX();
        double dy = goalPosition.getY() - follower.getPose().getY();
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

        // Calculate vertical distance (height difference)
        double verticalDistance = goalHeightInches - launcherHeightInches;

        // Calculate optimal launch angle and velocity using projectile motion physics
        // For a given distance and height, we solve: range = v^2 * sin(2θ) / g, height = v^2 * sin^2(θ) / (2g)
        // We use an iterative approach to find the best angle and velocity

        double optimalAngleDegrees = calculateOptimalAngle(horizontalDistance, verticalDistance);
        double optimalVelocity = calculateRequiredVelocity(horizontalDistance, verticalDistance, optimalAngleDegrees);

        // Convert velocity (inches/sec) to motor velocity (ticks/sec)
        // Linear velocity = (RPM × wheel_diameter × π) / 60
        // Angular velocity (RPM) = (linear_velocity × 60) / (wheel_diameter × π)
        // Ticks/sec = (RPM × ticks_per_rev) / 60
        // Therefore: ticks/sec = (linear_velocity × ticks_per_rev) / (wheel_diameter × π)

        double wheelCircumference = launcherWheelDiameterInches * Math.PI;
        double requiredVelocityTicksPerSec = (optimalVelocity / launchVelocityFactor) * launcherTicksPerRev / wheelCircumference;
        requiredVelocityTicksPerSec = Range.clip(requiredVelocityTicksPerSec, minLauncherVelocity, maxLauncherVelocity);

        // Convert angle (degrees) to servo position
        double angleRange = launchAngleMaxDegrees - launchAngleMinDegrees;
        double normalizedAngle = (optimalAngleDegrees - launchAngleMinDegrees) / angleRange;
        double servoPosition = servoAngleMin + normalizedAngle * (servoAngleMax - servoAngleMin);
        servoPosition = Range.clip(servoPosition, servoAngleMin, servoAngleMax);

        // Apply calculated values
        launcher.setVelocity(requiredVelocityTicksPerSec);
        launchAngleServo.setPosition(servoPosition/360);

        // Telemetry for debugging
        panelsTelemetry.addData("Distance to goal", String.format("%.1f in", horizontalDistance));
        panelsTelemetry.addData("Height difference", String.format("%.1f in", verticalDistance));
        panelsTelemetry.addData("Optimal angle", String.format("%.1f deg", optimalAngleDegrees));
        panelsTelemetry.addData("Required velocity", String.format("%.1f in/s", optimalVelocity));
        panelsTelemetry.addData("Launcher velocity", String.format("%.0f ticks/s", requiredVelocityTicksPerSec));
        panelsTelemetry.addData("Servo position", String.format("%.3f", servoPosition));
        panelsTelemetry.addData("Current velocity", String.format("%.0f ticks/s", launcher.getVelocity()));

    }

    /**
     * Calculate optimal launch angle for given distance and height
     * Uses a simplified approach: for maximum range with height constraint
     */
    private double calculateOptimalAngle(double horizontalDistance, double verticalDistance) {
        // For flat trajectory (verticalDistance ≈ 0), optimal angle is 45°
        // For upward trajectory, we need a higher angle
        // Simple heuristic: angle = 45° + adjustment based on height/distance ratio

        double baseAngle = 45.0;
        double heightRatio = verticalDistance / Math.max(horizontalDistance, 1.0);
        double angleAdjustment = Math.toDegrees(Math.atan(heightRatio)) * 0.5;

        double optimalAngle = baseAngle + angleAdjustment;

        // Clamp to servo range
        optimalAngle = Range.clip(optimalAngle, launchAngleMinDegrees, launchAngleMaxDegrees);

        return optimalAngle;
    }

    /**
     * Calculate required launch velocity for given distance, height, and angle
     * Uses projectile motion equation: v = sqrt(g * d^2 / (2 * cos^2(θ) * (d*tan(θ) - h)))
     */
    private double calculateRequiredVelocity(double horizontalDistance, double verticalDistance, double angleDegrees) {
        double angleRad = Math.toRadians(angleDegrees);
        double cosTheta = Math.cos(angleRad);
        double tanTheta = Math.tan(angleRad);

        // v^2 = g * d^2 / (2 * cos^2(θ) * (d*tan(θ) - h))
        double denominator = 2.0 * cosTheta * cosTheta * (horizontalDistance * tanTheta - verticalDistance);

        if (denominator <= 0) {
            // Invalid trajectory - return a default high velocity
            return 200.0; // inches per second
        }

        double velocitySquared = (gravity * horizontalDistance * horizontalDistance) / denominator;

        if (velocitySquared < 0) {
            return 200.0; // fallback
        }

        return Math.sqrt(velocitySquared);
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
