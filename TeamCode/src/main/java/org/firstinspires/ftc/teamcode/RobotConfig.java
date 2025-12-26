package org.firstinspires.ftc.teamcode;

public class RobotConfig {
    public double P, I, D, F;
    public int goalSpeed;
    public double goalAngle;
    public int middleSpeed;
    public double middleAngle;
    public int backSpeed;
    public double backAngle;
    public int juggleSpeed;
    public double juggleAngle;

    public static RobotConfig getForRobot(String robotName) {
        RobotConfig config = new RobotConfig();
        
        if ("RobotA".equalsIgnoreCase(robotName)) {
            config.P = 203; config.I = 1.001; config.D = 0.0015; config.F = 0.1;
            config.goalSpeed = 1080; config.goalAngle = 14;
            config.middleSpeed = 1400; config.middleAngle = 31;
            config.backSpeed = 1720; config.backAngle = 38;
            config.juggleSpeed = 500; config.juggleAngle = 8;
        } else if ("RobotB".equalsIgnoreCase(robotName)) {
            // Adjust these for the second robot
            config.P = 180; config.I = 0.95; config.D = 0.0012; config.F = 0.12;
            config.goalSpeed = 1100; config.goalAngle = 15;
            config.middleSpeed = 1450; config.middleAngle = 32;
            config.backSpeed = 1800; config.backAngle = 40;
            config.juggleSpeed = 550; config.juggleAngle = 9;
        } else {
            // Default Fallback
            config.P = 200; config.I = 1.0; config.D = 0.001; config.F = 0.1;
            config.goalSpeed = 1000; config.goalAngle = 10;
        }
        return config;
    }
}
