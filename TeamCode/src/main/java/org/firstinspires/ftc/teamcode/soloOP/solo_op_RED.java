package org.firstinspires.ftc.teamcode.soloOP;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Solo Op Red")
public class solo_op_RED extends solo_op_MAIN {
    public void set_color(){
        color = "Red";
    }

    public int target_goal_tag() {
      return 24;
    }

    public void back_line_pos() {
        back_x = 0;
        back_y = 0;
    }

//    @Override
//    public Pose getStartPosition() {
//        return new Pose(118.8, 70.941, 0);
//    }
//
//    @Override
//    public PathChain pathToTargetPoint(double x, double y, double heading) {
//        return this.follower.pathBuilder()
//                .addPath(new BezierLine(this.follower.getPose(), new Pose(x, y, heading)))
//                .setLinearHeadingInterpolation(this.follower.getHeading(), heading)
//                .build();
//    }
}
