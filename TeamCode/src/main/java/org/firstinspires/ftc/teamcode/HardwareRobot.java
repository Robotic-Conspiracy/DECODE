package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * This class defines all the specific hardware for the robot.
 */
public class HardwareRobot {
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  backLeftMotor    = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor  backRightMotor   = null;
    public DcMotorEx launcher        = null;
    public Servo    bendyServoOne    = null;
    public CRServo  leftFeeder       = null;
    public CRServo  rightFeeder      = null;
    
    // Odometry / Pinpoint
    public GoBildaPinpointDriver pod = null;

    // Vision members
    public AprilTagProcessor aprilTagProcessor = null;
    public VisionPortal visionPortal = null;

    // Robot Configuration details
    public RobotConfig config = null;

    /* local members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareRobot(){
    }

    /**
     * Initialize standard Hardware interfaces
     * @param ahwMap The hardware map from the OpMode
     * @return A string summary of which hardware was found or missing
     */
    public String init(HardwareMap ahwMap) {
        return init(ahwMap, "RobotA"); // Default to RobotA
    }

    public String init(HardwareMap ahwMap, String robotName) {
        hwMap = ahwMap;
        config = RobotConfig.getForRobot(robotName);
        StringBuilder report = new StringBuilder("Hardware Init Report (" + robotName + "):\n");

        // --- Drive Motors ---
        frontLeftMotor = tryGet(DcMotor.class, report, "Front Left", "left_front_drive", "front_left_drive", "leftFront", "frontLeft");
        backLeftMotor = tryGet(DcMotor.class, report, "Back Left", "left_back_drive", "back_left_drive", "leftBack", "backLeft", "left_rear_drive", "rear_left_drive");
        frontRightMotor = tryGet(DcMotor.class, report, "Front Right", "right_front_drive", "front_right_drive", "rightFront", "frontRight");
        backRightMotor = tryGet(DcMotor.class, report, "Back Right", "right_back_drive", "back_right_drive", "rightBack", "backRight", "right_rear_drive", "rear_right_drive");

        if (frontLeftMotor != null) frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (backLeftMotor != null) backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontRightMotor != null) frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (backRightMotor != null) backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Launcher ---
        launcher = tryGet(DcMotorEx.class, report, "Launcher", "launcher", "shoot", "flywheel");
        if (launcher != null && config != null) {
            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setVelocityPIDFCoefficients(config.P, config.I, config.D, config.F);
        }

        // --- Servos ---
        bendyServoOne = tryGet(Servo.class, report, "Bendy Servo", "bendy_servo_1", "bendy_servo", "arm_servo", "tilt_servo");
        leftFeeder = tryGet(CRServo.class, report, "Left Feeder", "left_feeder", "feeder_left");
        rightFeeder = tryGet(CRServo.class, report, "Right Feeder", "right_feeder", "feeder_right");

        if (leftFeeder != null) leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFeeder != null) rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // --- Pinpoint Pod ---
        try {
            pod = hwMap.getAll(GoBildaPinpointDriver.class).get(0);
            report.append("  [OK] Pinpoint Pod found\n");
        } catch (Exception e) {
            report.append("  [MISSING] Pinpoint Pod\n");
        }

        return report.toString();
    }

    private <T> T tryGet(Class<T> classOrInterface, StringBuilder report, String friendlyName, String... names) {
        for (String name : names) {
            try {
                T device = hwMap.get(classOrInterface, name);
                if (device != null) {
                    report.append("  [OK] ").append(friendlyName).append(" found as '").append(name).append("'\n");
                    return device;
                }
            } catch (Exception ignored) { }
        }
        report.append("  [MISSING] ").append(friendlyName).append("\n");
        return null;
    }

    public void initVision(AprilTagProcessor processor) {
        this.aprilTagProcessor = processor;
        try {
            WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcamName)
                    .addProcessor(aprilTagProcessor)
                    .build();
        } catch (Exception e) {
            visionPortal = null;
        }
    }

    public void setDrivePower(double fl, double bl, double fr, double br) {
        if (frontLeftMotor != null) frontLeftMotor.setPower(fl);
        if (backLeftMotor != null) backLeftMotor.setPower(bl);
        if (frontRightMotor != null) frontRightMotor.setPower(fr);
        if (backRightMotor != null) backRightMotor.setPower(br);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (frontLeftMotor != null) frontLeftMotor.setZeroPowerBehavior(behavior);
        if (backLeftMotor != null) backLeftMotor.setZeroPowerBehavior(behavior);
        if (frontRightMotor != null) frontRightMotor.setZeroPowerBehavior(behavior);
        if (backRightMotor != null) backRightMotor.setZeroPowerBehavior(behavior);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        if (frontLeftMotor != null) frontLeftMotor.setMode(mode);
        if (backLeftMotor != null) backLeftMotor.setMode(mode);
        if (frontRightMotor != null) frontRightMotor.setMode(mode);
        if (backRightMotor != null) backRightMotor.setMode(mode);
    }
}
