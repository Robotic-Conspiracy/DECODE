package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE 3 BACK", group = "Autonomous")
public class back3blue extends back3main {
    int backlineAngle = 110;
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
        paths.parkStart = new Pose(58, 16);
        paths.parkEnd = new Pose(36, 10);
        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;
        paths.headPark1 = backlineAngle;
        paths.headPark2 = 0;
    }
}
