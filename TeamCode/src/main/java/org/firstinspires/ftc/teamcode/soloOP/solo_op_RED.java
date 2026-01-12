package org.firstinspires.ftc.teamcode.soloOP;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Solo Op Red")
@Configurable
public class solo_op_RED extends solo_op_MAIN {
    public void set_color(){
        color = "Red";
    }

    public int target_goal_tag() {
      return 24;
    }

    @Override
    public Pose set_goal_position() {
        return new Pose(131.1, 136.7);
    }
}
