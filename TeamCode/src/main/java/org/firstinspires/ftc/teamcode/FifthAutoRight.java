package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Fifth Auto Right (Red)")
public class FifthAutoRight extends FifthAuto {
    
    @Override
    public void init() {
        super.init();
        color = "Red"; // Set color context
    }

    @Override
    public void move() {
        drive(-0.25, 0, 0);
    }

    @Override
    public void rotate() {
        drive(0, 0, 0.15);
    }

    @Override
    public void strafe() {
        drive(0, 0.25, 0);
    }
}
