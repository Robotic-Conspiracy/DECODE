package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Goal", group = "Autonomous")
public class GoalBlue extends GoalMain {
    int GoalAimAngle = 135;
    public void set_starting_pose(){
        starting_pose_x = 17;
        starting_pose_y = 119;
        starting_pose_heading = 142;
        //follower.setStartingPose(new Pose(63, 8, Math.toRadians(90)));
    }
    public void set_color(){
        String color = "blue";
        int tagToAim = 20;
        paths.shootPreloadStart = new Pose(17.5, 119);
        paths.shootPreloadEnd = new Pose(23, 119);
        paths.parkStart = new Pose(23, 119);
        paths.parkEnd = new Pose(36, 132);
        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = GoalAimAngle;
        paths.headPark1 = GoalAimAngle;
        paths.headPark2 = 0;
    }
}
