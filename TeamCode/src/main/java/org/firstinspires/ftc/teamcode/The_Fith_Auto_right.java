package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import kotlin.NotImplementedError;

@Autonomous(name = "Blue no shooting")
public class The_Fith_Auto_right extends The_Fith_auto{
    @Override
    public void move() {
        leftFrontDrive.setPower(-1);
        leftBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(-1);
    }

    @Override
    public void rotate() {
        throw new NotImplementedError();
    }
}
