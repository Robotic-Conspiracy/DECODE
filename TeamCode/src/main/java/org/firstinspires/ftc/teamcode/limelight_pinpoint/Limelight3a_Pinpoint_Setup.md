# Limelight 3a and goBilda Pinpoint Hardware & Configuration Guide

This guide outlines the steps required to integrate the **Limelight 3a** vision sensor and the **goBilda Pinpoint Odometry Computer** with the DECODE robot codebase using **Pedro Pathing**.

## 1. Hardware Setup

### Limelight 3a
*   **Mounting**: Mount the Limelight 3a at the front or top of the robot with a clear view of the field (specifically AprilTags or the Goal). Note its offset from the robot's center (X, Y, Z) and its pitch angle.
*   **Power**: Use a high-current 12V port on the Rev Power Distribution Hub (PDH). Limelight 3a requires stable power, especially during peak processing.
*   **Data**: 
    *   Connect the Limelight to the Control Hub via an Ethernet cable. 
    *   If using multiple Ethernet devices, use an Ethernet Switch (like the REV Control Hub Switch).
    *   Alternatively, use a USB-to-Ethernet adapter plugged into the Control Hub's USB 3.0 port.

### goBilda Pinpoint Odometry Computer
*   **Mounting**: The Pinpoint computer should be mounted flat and securely to the robot chassis. Ensure the orientation matches one of the supported modes (horizontal/vertical).
*   **Encoder Connection**: Plug your X and Y odometry pods into the dedicated ports on the Pinpoint computer.
*   **I2C Connection**: Connect the Pinpoint's I2C cable to an I2C port on the Control Hub (ideally a dedicated bus to minimize latency).
*   **Calibration**: Perform the physical calibration of the pods using the on-device buttons or the configuration tool to ensure correct distance per tick.

---

## 2. Limelight Configuration (Web Interface)

1.  Connect your laptop to the robot's Wi-Fi.
2.  Navigate to `http://10.XX.YY.11:5801` (replace XX.YY with your team number).
3.  **Pipeline Setup**:
    *   Create a pipeline for AprilTag detection (Field Localization).
    *   Create a pipeline for "Neural" or "Color" detection of the Goal if not using Tags.
4.  **IP Address**: Set a static IP for the Limelight (e.g., `10.XX.YY.11`) to ensure consistent connectivity.

---

## 3. goBilda Pinpoint Driver Setup

1.  Download the `GoBildaPinpointDriver.java` from the goBilda website or GitHub.
2.  Place the driver in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
3.  In the robot configuration on the Driver Station, add the I2C device on the correct port and name it `"odometry"`.

---

## 4. Pedro Pathing Integration

To use the Pinpoint as the localizer for Pedro Pathing:
1.  Update `pedroPathing/Constants.java` to initialize a `PinpointLocalizer`.
2.  Pass the Pinpoint hardware object to the localizer.
3.  Configure offsets (X, Y) in the `Constants.java` file.
