package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import kotlin.NotImplementedError;

@Autonomous(name = "Blue Shoot from back")
public class The_Fith_Auto_left extends The_Fith_auto{
    @Override
    public void move() {
        color = "Blue";
        double forward  = -0.25;
        double strafe = 0;
        double rotate = 0;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward - strafe - rotate)/denominator);
        leftBackDrive.setPower((forward + strafe - rotate)/denominator);
        rightFrontDrive.setPower((forward + strafe + rotate)/denominator);
        rightBackDrive.setPower((forward - strafe + rotate)/denominator);
    }

    @Override
    public void rotate() {
        double forward  = 0;
        double strafe = 0;
        double rotate = -0.25;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward - strafe - rotate)/denominator);
        leftBackDrive.setPower((forward + strafe - rotate)/denominator);
        rightFrontDrive.setPower((forward + strafe + rotate)/denominator);
        rightBackDrive.setPower((forward - strafe + rotate)/denominator);

    }
}
