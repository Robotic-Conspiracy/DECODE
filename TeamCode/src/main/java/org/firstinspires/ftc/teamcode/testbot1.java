package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testbot1 extends OpMode {
    private CRServo grabbyServo1;
    private CRServo grabbyServo2;
    private CRServo grabbyServo3;
    @Override
    public void init() {
        grabbyServo1 = hardwareMap.get(CRServo.class, "servo 1");
        grabbyServo2 = hardwareMap.get(CRServo.class, "servo 2");
        grabbyServo3 = hardwareMap.get(CRServo.class, "servo 3");
        grabbyServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        grabbyServo2.setDirection(DcMotorSimple.Direction.REVERSE);
        grabbyServo3.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        grabbyServo1.setPower(1);
        grabbyServo2.setPower(1);
        grabbyServo3.setPower(1);

    }
}
