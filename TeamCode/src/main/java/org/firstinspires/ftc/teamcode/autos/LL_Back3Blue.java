package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LL BLUE 3 BACK", group = "Autonomous")
public class LL_Back3Blue extends LL_Back3Main {
    int backlineAngle = 109;

    @Override
    public void set_starting_pose(){
        starting_pose_x = 63;
        starting_pose_y = 8;
        starting_pose_heading = 90;
    }

    @Override
    public void set_color(){
        color = "blue";
        targetTxOffset = 0.0; // Adjust if needed for blue blue goal side
        
        paths.shootPreloadStart = new com.pedropathing.geometry.Pose(63.000, 8.000);
        paths.shootPreloadEnd = new com.pedropathing.geometry.Pose(58, 16);
        paths.parkStart = new com.pedropathing.geometry.Pose(58, 16);
        paths.parkEnd = new com.pedropathing.geometry.Pose(36, 10);
        
        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;
        paths.headPark1 = backlineAngle;
        paths.headPark2 = 0;
    }
}
