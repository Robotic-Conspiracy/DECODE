package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "drive right auto red")
public class The_Third_Auto extends OpMode {
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private GoBildaPinpointDriver pod;
    private states state = states.NOT_READY;
    private int timesShot = 0;

    ElapsedTime feedTimer = new ElapsedTime();


    public static double P = 500;
    public static double I = 0.2;
    public static double D = 0;
    public static double F = 0;




    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        pod.resetPosAndIMU();
    }

    @Override
    public void loop() {
        launcher.setVelocity(1150);
        telemetry.addData("Position", pod.getPosition());
        telemetry.addData("Velocity", launcher.getVelocity());
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        telemetry.addLine("Version 1.0.3");
        pod.update();
        switch(state) {
            case NOT_READY:
                leftFrontDrive.setPower(-1);
                rightFrontDrive.setPower(1);
                leftBackDrive.setPower(1);
                rightBackDrive.setPower(-1);


                if (Math.abs(pod.getPosY()) >= 50) {
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    state = states.SPIN_UP;
                }

                break;
            case SPIN_UP:
                if (feedTimer.seconds() > 1) {
                    state = (launcher.getVelocity() >= 1080 && launcher.getVelocity() <= 1200) ? states.LAUNCH : state;
                }

                break;
            case LAUNCH:
                leftFeeder.setPower(1);
                rightFeeder.setPower(1);
                feedTimer.reset();
                state = states.LAUNCHING;
                break;
            case LAUNCHING:
                telemetry.addData("Feed time", feedTimer.seconds());
                if(timesShot <= 3){
                    if (feedTimer.seconds() > 0.2) {
                        state = states.SPIN_UP;
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        timesShot += 1;
                    }
                } else {
                    state = states.REVERSING;
                }

                break;
            case REVERSING:
                if(Math.abs(pod.getPosX()) <= 150){
                    System.out.println("Back");
                    leftFrontDrive.setPower(1);
                    rightFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightBackDrive.setPower(1);
                } else if (Math.abs(pod.getPosY()) <= 75*4) {//not large enough value
                    System.out.println("Right");
                    leftFrontDrive.setPower(-1);
                    rightFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightBackDrive.setPower(-1);

                } else {
                    state = states.DONE;
                }
                break;
            case DONE:
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                break;

        }

        telemetry.addData("state", state);

    }

    private enum states {
        NOT_READY,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        REVERSING,
        DONE
    }
}


