package org.firstinspires.ftc.teamcode.limelight_pinpoint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3a;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.soloOP.solo_op_MAIN;

/**
 * TeleOp OpMode that extends solo_op_MAIN to include Limelight-based Auto-Aiming.
 * 
 * Features:
 * 1. Alliance Selection (X=Blue, B=Red) during init_loop.
 * 2. Uses all controls and hardware mappings from soloOP.
 * 3. Adds Limelight precision aiming when holding Button A.
 */
@TeleOp(name = "TeleOp Auto-Aim Limelight", group = "TeleOp")
@Configurable
public class AutoAimLimelight extends solo_op_MAIN {

    @Override
    public void set_color() {
        // Default to blue if not yet set
        if (color == null || color.equals("None")) {
            color = "blue";
        }
    }

    @Override
    public int target_goal_tag() {
        // Blue goal: 20, Red goal: 24 (tags verified from solo_op_BLUE/RED)
        if (color == null) return 20;
        return color.equalsIgnoreCase("blue") ? 20 : 24;
    }

    @Override
    public Pose set_goal_position() {
        // Goal positions verified from solo_op_BLUE/RED
        if (color == null) return new Pose(12.2, 136.7);
        return color.equalsIgnoreCase("blue") ? new Pose(12.2, 136.7) : new Pose(131.1, 136.7);
    }

    @Override
    public void init() {
        super.init();
        telemetry.addLine("AutoAimLimelight initialized (inherits solo_op_MAIN)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Selection of alliance color before start
        // X = Blue, B = Red
        if (gamepad1.x) {
            color = "blue";
        } else if (gamepad1.b) {
            color = "red";
        }
        
        // Keep goal position updated based on selection
        goalPosition = set_goal_position();
        
        telemetry.addData("Selected Alliance", color.toUpperCase());
        telemetry.addLine("Press [X] for BLUE | Press [B] for RED");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Execute solo_op_MAIN's standard loop logic which now includes 
        // the button mappings and aiming techniques from OpmodeConstants.
        super.loop();
    }
}
