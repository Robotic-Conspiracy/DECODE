package org.firstinspires.ftc.teamcode.soloOP;
import com.bylazar.configurables.annotations.Configurable;
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
}
