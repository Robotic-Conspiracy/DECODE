package org.firstinspires.ftc.teamcode.soloOP;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Configurable
public class aimTestingRed extends aimingTest{
    @Override
    public Pose set_goal_position() {
        return new Pose(131.1, 136.7);
    }
}
