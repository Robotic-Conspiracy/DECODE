package org.firstinspires.ftc.teamcode.soloOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Solo Op Red")
public class solo_op_RED extends solo_op_MAIN {
    public void set_color(){
        color = "Red";
    }

    public int target_goal_tag() {
      return 24;
    }

    @Override
    void setSpeedConstants() {

    }

    @Override
    void setAngleConstants() {

    }
}
