package org.firstinspires.ftc.teamcode.soloOP;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Solo Op Blue")
public class solo_op_BLUE extends solo_op_MAIN {
    @Override
    public void set_color(){
        color = "blue";
    }

    public int target_goal_tag() {
      return 20;
    }
}
