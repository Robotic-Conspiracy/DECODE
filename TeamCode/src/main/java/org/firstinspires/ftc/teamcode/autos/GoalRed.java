package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Goal", group = "Autonomous")
public class GoalRed extends GoalMain {
    // Mirrored from blue GoalAimAngle = 135 -> 180 - 135 = 45
    int GoalAimAngle = 38; // mirrored aim angle

    public void set_starting_pose(){
        // Mirrored from blue starting_pose_x = 17 -> 144 - 17 = 127
        starting_pose_x = 127;
        starting_pose_y = 119;
        // Mirror heading: 180 - 142 = 38
        starting_pose_heading = 38;
    }

    public void set_color(){
        String color = "red";
        int tagToAim = 24; // keep red AprilTag id

        // Mirrored from blue across x = 72 -> x' = 144 - x
        // Blue shootPreloadStart was (17.5, 119) -> Red is (126.5, 119)
        paths.shootPreloadStart = new Pose(127, 119);
        // Blue shootPreloadEnd was (23, 119) -> Red is (121, 119)
        paths.shootPreloadEnd = new Pose(121, 119);

        // Blue parkStart was (23, 119) -> Red is (121, 119)
        paths.parkStart = new Pose(121, 119);
        // Blue parkEnd was (36, 132) -> Red is (108, 132)
        paths.parkEnd = new Pose(108, 132);

        paths.headShootPreload1 = 90;
        paths.headShootPreload2 = GoalAimAngle;
        paths.headPark1 = GoalAimAngle;
        paths.headPark2 = 180; // mirrored from blue 0 -> 180
    }
}
