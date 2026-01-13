# Limelight + Pinpoint Integration Guide

This directory contains the logic for advanced vision-assisted TeleOp using the Limelight 3a and goBilda Pinpoint.

## Comparison of Auto-Aiming Techniques

The robot supports three distinct aiming methods, switchable via the settings in `OpmodeConstants.java`:

| Technique | Method | Button (Default) | Best For... |
| :--- | :--- | :--- | :--- |
| **Limelight Precision** | Vision-based (TX offset) | **Button [A]** | Final pixel-perfect alignment before shooting. |
| **Odometry Snap** | Coordinate-based (Pose) | **Button [B]** | Rapidly turning to face the goal from any position. |
| **AprilTag Lock** | Relative Bearing | **Right Trigger** | Positioning relative to the backline/tags. |

---

## Connection & Setup Steps

### 1. Physical Connectivity
- Connect the **Limelight 3a** directly to the Control Hub's **USB 3.0 port** (the blue one) using a high-quality USB-C to USB-A cable.
- **Note**: No Ethernet switches or adapters are required for the current direct-USB configuration.
- Ensure the **Limelight 3a** is receiving stable 12V power from a high-current port on the REV PDH.

### 2. Tuning
- Gains for the Limelight precision aiming (`Limelight_P` and `Limelight_MIN_POWER`) are located in `OpmodeConstants.java`.
- If the robot oscillates while aiming, decrease `Limelight_P`.
- If the robot stops too early or doesn't move, increase `Limelight_MIN_POWER`.


### 2. Hardware Mapping
In your Robot Configuration on the Driver Station:
- **Limelight**: Name it `limelight` (type: Limelight 3A).
- **Pinpoint**: Name it `pinpoint` (type: SparkFun/goBilda Pinpoint).

### 3. Operation
1. Select the `TeleOp Auto-Aim Limelight` OpMode.
2. During Init, press **X (Blue)** or **B (Red)** to select your alliance.
3. Once started, drive normally.
4. When ready to shoot, hold **[A]** while facing generally toward the goal; the robot will snap its rotation to the goal center using vision.

---

## Useful Resources

### Official Documentation
- [Limelight 3a Hardware Guide](https://docs.limelightvision.io/docs/docs-limelight/getting-started/hardware-setup)
- [Limelight FTC Coordinate Systems](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems)
- [goBilda Pinpoint Support](https://www.gobilda.com/pinpoint-odometry-computer/)
- [Pedro Pathing Documentation](https://pedropathing.com/)

### Debugging Tools
- [Limelight Web Dashboard](http://10.28.139.11:5801) (Replace with your team's IP)
- [FTC Dashboard](http://192.168.43.1:8080/dash) - Use this to tune `Limelight_P` in real-time.
