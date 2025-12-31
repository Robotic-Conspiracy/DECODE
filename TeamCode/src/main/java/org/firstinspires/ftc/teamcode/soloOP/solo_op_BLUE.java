package org.firstinspires.ftc.teamcode.soloOP;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    public void back_line_pos() {
        back_x = 0;
        back_y = 0;
    }

    @Override
    public Pose getStartPosition() {
        return new Pose(25.2, 70.941, Math.toRadians(180));
    }

    @Override
    public PathChain pathToTargetPoint(double x, double y, double heading) {
        return this.follower.pathBuilder()
                .addPath(new BezierLine(this.follower.getPose(), new Pose(x, y, heading)))
                .setLinearHeadingInterpolation(this.follower.getHeading(), heading)
                .build();
    }

}
