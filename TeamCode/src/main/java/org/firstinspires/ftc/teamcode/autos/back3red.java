package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED 3 BACK", group = "Autonomous")
public class back3red extends back3main {
    int backlineAngle = 70; // Mirrored from blue 110 (180 - 110 = 70, adjusted to 67 to match other red autos)

    public void set_starting_pose(){
        // Mirrored from blue: x' = 144 - 63 = 81
        starting_pose_x = 81;
        starting_pose_y = 8;
        starting_pose_heading = 90;
    }

    public void set_color(){
        String color = "red";
        int tagToAim = 24; // Red side AprilTag

        // Mirrored from blue across x = 72 -> x' = 144 - x
        // Blue shootPreloadStart was (63, 8) -> Red is (81, 8)
        paths.shootPreloadStart = new Pose(81.000, 8.000);
        // Blue shootPreloadEnd was (58, 16) -> Red is (86, 16)
        paths.shootPreloadEnd = new Pose(86, 16);

        // Blue parkStart was (58, 16) -> Red is (86, 16)
        paths.parkStart = new Pose(86, 16);
        // Blue parkEnd was (36, 10) -> Red is (108, 10)
        paths.parkEnd = new Pose(108, 10);

        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = backlineAngle;
        paths.headPark1 = backlineAngle;
        paths.headPark2 = 180; // Mirrored from blue 0 -> 180
    }
}
