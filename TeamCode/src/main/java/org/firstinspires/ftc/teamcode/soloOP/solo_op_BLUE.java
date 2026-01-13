package org.firstinspires.ftc.teamcode.soloOP;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Solo Op Blue")
@Configurable
public class solo_op_BLUE extends solo_op_MAIN {
    @Override
    public void set_color(){
        color = "blue";
    }

    public int target_goal_tag() {
      return 20;
    }

    @Override
    public Pose set_backline_position() {
        return new Pose(58,16);
    }
    public int set_backline_angle(){return 110;}
    public Pose set_gate_position() {
        return new Pose(10.870, 61.913);
    }
    public int set_gate_angle(){return 147;}
    public Pose set_human_position() {
        return new Pose(133,11);
    }
    public int set_human_angle(){return 0;}

    public Pose set_goal_position() {
        return new Pose(12.2, 136.7);
    }
}
