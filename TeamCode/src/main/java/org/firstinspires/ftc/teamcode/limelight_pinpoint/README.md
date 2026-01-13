# Limelight + Pinpoint Integration Guide

This directory contains the logic for advanced vision-assisted TeleOp using the Limelight 3a and goBilda Pinpoint.

## Comparison of Auto-Aiming Techniques

The robot supports three distinct aiming methods, switchable via the settings in `OpmodeConstants.java`:

| Technique | Method | Button (Default) | Best For... |
| :--- | :--- | :--- | :--- |
| **Limelight Precision** | Vision-based (TX offset) | **Button [A]** | Final pixel-perfect alignment before shooting. |
| **Odometry Snap** | Coordinate-based (Pose) | **Button [B]** | Rapidly turning to face the goal from any position. |
| **AprilTag Lock** | Relative Bearing | **Right Trigger** | Positioning relative to the backline/tags. |

NOTE: Buttons can be re-mapped in OpmodeConstants.java
---

## Connection & Setup Steps

### 1. Physical Connectivity
- Connect the **Limelight 3a** directly to the Control Hub's **USB 3.0 port** (the blue one) using a high-quality USB-C to USB-A cable.
- Ensure the **Limelight 3a** is receiving stable 12V power from a high-current port on the REV PDH.

### 2. Tuning
- Gains for the Limelight precision aiming (`Limelight_P` and `Limelight_MIN_POWER`) are located in `OpmodeConstants.java`.
- If the robot oscillates while aiming, decrease `Limelight_P`.
- If the robot stops too early or doesn't move, increase `Limelight_MIN_POWER`.


### 2. Hardware Mapping
In your Robot Configuration on the Driver Station:
- **Limelight**: Name it `limelight` (type: Limelight 3A).
- **Pinpoint**: Name it `odometry` (type: goBilda Pinpoint).

### 3. Operation
1. Select the `TeleOp Auto-Aim Limelight` OpMode.
2. During Init, press **X (Blue)** or **B (Red)** to select your alliance.
3. Once started, drive normally.
4. When ready to shoot, hold **[A]** while facing generally toward the goal; the robot will snap its rotation to the goal center using vision.

---

## Autonomous Limelight Integration (`LL_` Files)

For advanced autonomous performance, use the `LL_` prefixed versions of the autonomous OpModes (e.g., `LL_Back3Blue`).

### How it Works: "Hybrid Alignment"
The `LL_` autos use a two-stage approach to alignment:
1.  **Macro Movement (Pedro Pathing)**: The robot travels to the shooting position using standard Pedro Pathing (`shootPreload`, `shootPickup`).
2.  **Micro Alignment (Limelight)**: Upon reaching the target pose, the robot enters `case 100`. In this state, it performs a high-frequency P-loop using the Limelight's horizontal error (`TX`) to ensure the heading is pixel-perfect before firing.

### Fail-Safes & Non-Blocking Logic
The Limelight logic is designed never to get the robot stuck:
- **`LIMELIGHT_AUTO_LOCK_TIMEOUT`**: If the robot hasn't reached the threshold within this time (default 1.5s), it proceeds anyway.
- **Target Validity Check**: If the Limelight does not see a valid target immediately, it skips alignment and falls back to the original Pedro Pathing heading.
- **Adjustable Offsets**: Subclasses can override `targetTxOffset` to shift the aim slightly left or right of center to compensate for field-specific goals.

### Integration Best Practices
1.  **Decoupling Constraints**: When vision alignment starts, we switch the follower to "TeleOp mode" (`setTeleOpMovement(0,0,turn)`). This allows the vision logic to control the rotation without fighting Pedro's internal path constraints.
2.  **Puning for Surface**: The alignment loop uses `Limelight_P` from `OpmodeConstants.java`. For autonomous, you may want a slightly more aggressive P-value than TeleOp to minimize cycle time.
3.  **Threshold vs. Time**: Balance `LIMELIGHT_AUTO_LOCK_THRESHOLD` and `TIMEOUT`. A tighter threshold (0.2°) increases accuracy but may trigger a timeout more often on vibrating intake systems.

---

## Student Concept Guide: Hybrid Navigation

This guide explains the "Why" and "How" behind the `LL_` Autonomous files for students and mentors.

### 1. Macro vs. Micro Navigation
In robotics, we often split movement into two phases:
- **Macro (The Journey)**: Using Odometry (Pinpoint) and Pathing (Pedro) to move across the field. This is fast but can drift by an inch or two over time.
- **Micro (The Aim)**: Using Vision (Limelight) once we are in position. This ignores where the robot *thinks* it is on the field and looks directly at the goal for "pixel-perfect" truth.

### 2. The Concept of "Decoupling"
A common mistake is trying to run a Path Follower and a Vision Loop at the same time. This is like having two people trying to steer the same car.
- **Why we decouple**: Pedro Pathing has an internal PID controller that fights to keep the robot on a specific heading. If the Limelight tries to rotate the robot slightly differently, the two controllers will fight, causing jitter or "hunting."
- **How we do it**: In `case 100`, we check `!follower.isBusy()`. We then use `follower.setTeleOpMovement(0, 0, turnPower)`. 
  - This effectively puts Pedro's "Autonomous Brain" into neutral. 
  - It allows us to manually control the motors using Limelight data. 
  - When we move to the next `case`, we give Pedro a new path, and its Autonomous Brain automatically re-engages.

### 3. Understanding the State Machine
The `autonomousPathUpdate()` function is a **State Machine**. Think of it as a checklist:
- **Case 1**: Drive to target. (Check? Move to 100)
- **Case 100**: Align with Limelight. (Check? Move to 101)
- **Case 101**: Shoot! (Check? Move to 2)
- **Case 2**: Drive to Park.

### 4. Limelight Pipelines & MegaTag
- **Pipelines**: We use `pipelineSwitch(0)`. Pipeline 0 is configured for AprilTags. It provides `TX` (Horizontal Offset), which is the fastest way to align.
- **MegaTag**: While we use `TX` for the final "Micro" snap, the **MegaTag** logic (seen in TeleOp) is used to update the global Pose. In Auto, MegaTag is less critical because we are using the Goal as a direct visual target, but it's used in the background to keep the field map accurate.

### 5. Fail-Safes (Non-Blocking Logic)
Real-world conditions aren't perfect. A robot can get stuck if a sensor fails. We use two main fail-safes:
1. **The Timeout**: `LIMELIGHT_AUTO_LOCK_TIMEOUT` (1.5s) stops the robot from waiting forever if it can't see the target.
2. **Target Check**: `if (result == null || !result.isValid())` immediately skips alignment if the camera is blind.

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
