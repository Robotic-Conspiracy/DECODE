package org.firstinspires.ftc.teamcode.pedroPathing;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Set 1620 Motor to 1400 RPM")
public class motor_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");

        // Yellow Jacket 1620 RPM (3.7:1) encoder resolution
        double TPR = 103.8;  // ticks per rotation

        // Target RPM
        double targetRPM = 1400;

        // Convert RPM → ticks per second
        double targetTPS = (targetRPM / 60.0) * TPR;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(targetTPS);  // FTC SDK uses ticks/second

        waitForStart();

        while (opModeIsActive()) {

            // Read actual velocity in ticks per second
            double actualTPS = motor.getVelocity();

            // Convert back to RPM for telemetry
            double actualRPM = (actualTPS / TPR) * 60.0;

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", actualRPM);
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", actualTPS);
            telemetry.update();
        }
    }
}
