package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Fifth Auto Left")
public class FifthAutoLeft extends FifthAuto {
    
    @Override
    public void init() {
        super.init();
        color = "Blue"; // Set color context
    }

    @Override
    public void move() {
        drive(0.2, 0, 0);
    }

    @Override
    public void rotate() {
        drive(0, 0, 0.2);
    }

    @Override
    public void strafe() {
        drive(0, 0.2, 0);
    }
}
