package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED GOAL")

public class GoalRed extends GoalMain {
    @Override
    void move() {
        double forward  = 1;
        double strafe = -1;
        double rotate = 0;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward - strafe - rotate)/denominator);
        leftBackDrive.setPower((forward + strafe - rotate)/denominator);
        rightFrontDrive.setPower((forward + strafe + rotate)/denominator);
        rightBackDrive.setPower((forward - strafe + rotate)/denominator);
    }
}
