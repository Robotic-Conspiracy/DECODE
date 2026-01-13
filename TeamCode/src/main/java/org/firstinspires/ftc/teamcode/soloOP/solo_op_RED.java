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
    public Pose set_backline_position() {
        return new Pose(86, 16);
    }
    public int set_backline_angle(){return 70;}
    public Pose set_gate_position() {return new Pose(133.13, 63);}
    public int set_gate_angle(){return 57;}
    public Pose set_human_position() {
        return new Pose(11,11);
    }
    public int set_human_angle(){return 180;}
    public Pose set_goal_position() {
        return new Pose(131.1, 136.7);
    }
}
