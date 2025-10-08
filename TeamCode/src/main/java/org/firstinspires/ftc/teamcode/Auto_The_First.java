package org.firstinspires.ftc.teamcode;

//Road Runner Imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
@Autonomous
public class Auto_The_First extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;



    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private GoBildaPinpointDriver pod = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public static int X_Goal = 0;
    public static int Y_Goal = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

        //Commented Out while only testing drive
//
//        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
//        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
//
//
//        leftFeeder.setPower(STOP_SPEED);
//        rightFeeder.setPower(STOP_SPEED);
//
//        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */

//        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set pod to zero zero
        pod.resetPosAndIMU();

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        pod.update();
        telemetry.addData("x pos", pod.getPosX());
        telemetry.addData("y pos", pod.getPosY());
        telemetry.update();

        double forward = 0;
        double strafe = 0;
        double rotate = 0;
        double change_by = 0.2;

        double posX = pod.getPosX();
        double posY = pod.getPosY();

//        if (X_Goal > posX) {
//            strafe = change_by;
//        }
//
//        if (Y_Goal > posY) {
//            forward = change_by;
//        }

        if(X_Goal - 5 > posX){
            strafe = change_by;
        }
        else if (X_Goal + 5 < posX){
            strafe = -change_by;
        }

        if (Y_Goal - 5 > posY){
            forward = change_by;
        } else if (Y_Goal + 5 < posY){
            forward = -change_by;
        }

        mecanumDrive(forward, strafe, -rotate);
        telemetry.addData("Forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("Rotate", rotate);

    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
}
