package org.firstinspires.ftc.teamcode;
public class OpmodeConstants {
    public static final int TeleopBacklineSpeed = 2300;
    public static final int TeleopMidlineSpeed = 1940;
    public static final int TeleopGoalSpeed = 1500;
    public static final double TeleopMidAngle = 0.133333; //48/360.0;
    public static final double TeleopGoalAngle = 0.208333; //75/360.0;
    public static final double TeleopBacklineAngle = 0.14165; //50/360.0;

    public static final int AutoBacklineSpeed = 2240;
    public static final int AutoMidSpeed = 1820;
    public static final int AutoGoalSpeed = 1500;
    public static final double AutoMidAngle = 0.133333; //48/360.0;
    public static final double AutoGoalAngle = 0.202777; //73/360.0;
    public static final double AutoBacklineAngle = 0.136111; // 49/360.0;

    public static final double IntakeRampLaunchPos = 0.61;
    public static final double IntakeRampIntakePos = 0.84;
    public static final double FeedTimeSeconds = 0.15;

    public static final double Launcher_P = 80;
    public static final double Launcher_I = 0.002;
    public static final double Launcher_D = 0.1;
    public static final double Launcher_F = 11;
//hardware map names
    public static final String IntakeName = "intake";
    public static final String IntakeRampName = "intake ramp";
    public static final String IntakeStopperName = "intake stopper";
    public static final String AimServoName = "left twideler";
    public static final String LeftFeederName = "left_feeder";
    public static final String RightFeederName = "right_feeder";
    public static final String LauncherName = "launcher";

    public static final String FrontLeftMotor = "front_left_drive";
    public static final String FrontRightMotor = "front_right_drive";
    public static final String BackLeftMotor = "back_left_drive";
    public static final String BackRightMotor = "back_right_drive";

    public static final String IMUName = "imu";
    public static final String odometryName = "odometry";
    public static final String PresetLightName = "preset light";
    public static final String AimLightName = "aim light";
    public static final String FloodgateName = "floodgate";
    public static final String WebcamName = "Webcam 1";
    public static final String LimelightName = "limelight";
    public static final String PinpointName = "pinpoint";

    // Limelight Features
    public static boolean LIMELIGHT_PRESENT = true;
    public static boolean LIMELIGHT_POSE_CALIBRATION = true;

    // Limelight Tuning
    public static double Limelight_P = 0.04;
    public static double Limelight_MIN_POWER = 0.05;

    // --- AUTO-AIM CONTROL MAPPINGS ---
    /**
     * Aiming Technique 1: Limelight Precision
     * Uses the Limelight's horizontal offset (TX) for pixel-perfect alignment.
     * Best for: Final adjustment before shooting.
     */
    public static boolean isLimelightAimPressed(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        return gamepad.a;
    }

    /**
     * Aiming Technique 2: Odometry Snap
     * Uses field coordinates (Pose) to calculate the angle to the goal.
     * Best for: Quick turn-to-goal from any orientation.
     */
    public static boolean isOdometryAimPressed(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        return gamepad.b;
    }

    /**
     * Aiming Technique 3: AprilTag Locking
     * Uses AprilTag relative distance and bearing for alignment.
     * Best for: Positioning relative to the backline.
     */
    public static boolean isAprilTagAimPressed(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        return gamepad.right_trigger > 0.2;
    }

    // Limelight Camera Position (relative to robot center)
    public static double CAMERA_X_OFFSET = 220.0; // mm (Forward of center is postive)
    public static double CAMERA_Y_OFFSET = -4.0;  // mm (Left of center is positive)
    public static double CAMERA_Z_OFFSET = 300.0; // mm (Height from floor)
    public static double CAMERA_PITCH = 0.0;      // degrees
    public static double CAMERA_YAW = 0.0;        // degrees
    public static double CAMERA_ROLL = 0.0;       // degrees

}
