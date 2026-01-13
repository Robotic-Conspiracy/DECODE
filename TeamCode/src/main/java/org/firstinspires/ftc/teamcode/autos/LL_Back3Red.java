package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LL RED 3 BACK", group = "Autonomous")
public class LL_Back3Red extends LL_Back3Main {
    int backlineAngle = 73;

    @Override
    public void set_starting_pose(){
        starting_pose_x = 81;
        starting_pose_y = 8;
        starting_pose_heading = 90;
    }

    @Override
    public void set_color(){
        color = "red";
        targetTxOffset = 0.0; // Adjust if needed for red goal side
        
        // Mirrored from blue across x = 72 -> x' = 144 - x
        paths.shootPreloadStart = new com.pedropathing.geometry.Pose(81.000, 8.000);
        paths.shootPreloadEnd = new com.pedropathing.geometry.Pose(86, 16);
        paths.parkStart = new com.pedropathing.geometry.Pose(86, 16);
        paths.parkEnd = new com.pedropathing.geometry.Pose(108, 10);

        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;
        paths.headPark1 = backlineAngle;
        paths.headPark2 = 180;
    }
}
