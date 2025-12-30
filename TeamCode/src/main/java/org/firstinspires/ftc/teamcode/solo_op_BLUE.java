package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Solo Op Blue")
public class solo_op_BLUE extends solo_op_MAIN{
    @Override
    public void set_color(){
        color = "blue";
    }

    public int target_goal_tag() {
      return 20;
    }
    public void back_line_pos() {
        back_x = 0;
        back_y = 0;
    }
}
