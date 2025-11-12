package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue goal")

public class The_Fourth_Auto_Blue extends The_Fourth_Auto{
  @Override
  void move() {
    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(1);
    leftBackDrive.setPower(1);
    rightBackDrive.setPower(0);
  }
}
