package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MAIN AUTO RED")

public class The_Fourth_Auto_Red extends The_Fourth_Auto{
  @Override
  void move() {
    leftFrontDrive.setPower(1);
    rightFrontDrive.setPower(0);
    leftBackDrive.setPower(0);
    rightBackDrive.setPower(1);
  }
}
