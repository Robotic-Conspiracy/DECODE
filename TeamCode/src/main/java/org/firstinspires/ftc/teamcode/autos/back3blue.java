package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

public class back3blue extends back3main{
    public void set_starting_pose(){
        starting_pose_x = 63;
        starting_pose_y = 8;
        //follower.setStartingPose(new Pose(63, 8, Math.toRadians(90)));
    }
    public void set_color(){
        String color = "blue";
        int tagToAim = 20;
        paths.shootPreloadStart = new Pose(63.000, 8.000);
        paths.shootPreloadEnd = new Pose(58, 16);
    }
}
