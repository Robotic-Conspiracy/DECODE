package org.firstinspires.ftc.teamcode;
public class OpmodeConstants {
    // Launcher motor velocities (in Ticks Per Second) for TeleOp presets
    public static final int TeleopBacklineSpeed = 2300;
    public static final int TeleopMidlineSpeed = 1940;
    public static final int TeleopGoalSpeed = 1500;

    // Launcher servo positions (0.0 to 1.0) for TeleOp presets
    public static final double TeleopMidAngle = 0.133333; //48/360.0;
    public static final double TeleopGoalAngle = 0.208333; //75/360.0;
    public static final double TeleopBacklineAngle = 0.133333; //48/360.0;

    // Launcher motor velocities (in Ticks Per Second) for Autonomous presets
    public static final int AutoBacklineSpeed = 2300;
    public static final int AutoMidSpeed = 1820;
    public static final int AutoGoalSpeed = 1500;

    // Launcher servo positions (0.0 to 1.0) for Autonomous presets
    public static final double AutoMidAngle = 0.133333; //48/360.0;
    public static final double AutoGoalAngle = 0.202777; //73/360.0;
    public static final double AutoBacklineAngle = 0.133333; // 48/360.0;

    // Intake system mechanism positions and timings
    public static final double IntakeRampLaunchPos = 0.61; // Angle when launching
    public static final double IntakeRampIntakePos = 0.84; // Angle when picking up
    public static final double FeedTimeSeconds = 0.15;      // Time to spin feeders per shot

    // Advanced alignment configuration

    // Launcher Motor PIDF Coefficients for velocity stability
    public static final double Launcher_P = 80;
    public static final double Launcher_I = 0.002;
    public static final double Launcher_D = 0.1;
    public static final double Launcher_F = 11;

    // Hardware mapping names (strings used in the robot configuration)
    public static final String IntakeName = "intake";
    public static final String IntakeRampName = "intake ramp";
    public static final String IntakeStopperName = "intake stopper";
    public static final String AimServoName = "left twideler";
    public static final String LeftFeederName = "left_feeder";
    public static final String RightFeederName = "right_feeder";
    public static final String LauncherName = "launcher";

    // Drive base motor mapping names
    public static final String FrontLeftMotor = "front_left_drive";
    public static final String FrontRightMotor = "front_right_drive";
    public static final String BackLeftMotor = "back_left_drive";
    public static final String BackRightMotor = "back_right_drive";

    // Sensor and Peripheral mapping names
    public static final String IMUName = "imu";
    public static final String odometryName = "odometry";
    public static final String PresetLightName = "preset light";
    public static final String AimLightName = "aim light";
    public static final String FloodgateName = "floodgate";
    public static final String WebcamName = "Webcam 1";
}
