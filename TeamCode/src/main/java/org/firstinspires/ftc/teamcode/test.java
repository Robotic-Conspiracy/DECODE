package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class test extends OpMode {
    protected DcMotor leftFrontDrive;
    protected DcMotor rightFrontDrive;
    protected DcMotor leftBackDrive;
    protected DcMotor rightBackDrive;
    protected Servo angleThing;
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private GoBildaPinpointDriver pod;
    private states state = states.NOT_READY;
    private int timesShot = 0;

    private ElapsedTime feedTimer = new ElapsedTime();
    private ElapsedTime waitTimer = new ElapsedTime();
    protected String color = "None";

    private double targetVelocity = 1940;

    @Override
    public void init(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angleThing = hardwareMap.get(Servo.class, "bendy_servo_1");
        pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        pod.resetPosAndIMU();
        pod.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.RADIANS,0));
    }
    @Override
    public void start() {
        waitTimer.reset();
        System.out.println("reset wait timer");
    }
    @Override
    public void loop() {
        launcher.setVelocity(targetVelocity);
        launcher.setVelocityPIDFCoefficients(203, 1.001, 0.0015, 0.1);
        angleThing.setPosition(38/360.0);
        pod.update();
        telemetry.addData("velocity ", launcher.getVelocity());
        telemetry.addData("position x", pod.getPosX(DistanceUnit.MM));
        telemetry.addData("position y", pod.getPosY(DistanceUnit.MM));
        telemetry.addData("angle", pod.getHeading(AngleUnit.DEGREES));
        telemetry.addData("state", state);

        telemetry.update();
//        telemetry.addData("front left wheel", leftFrontDrive.getPower());
//        telemetry.addData("front right wheel", rightFrontDrive.getPower());
//        telemetry.addData("back left wheel", leftBackDrive.getPower());
//        telemetry.addData("back right wheel", rightBackDrive.getPower());

        //ToDo: finish setting up the state machine
        switch(state) {
            case NOT_READY:
                if(waitTimer.seconds() >= 1) {
                    double angle = pod.getHeading(AngleUnit.DEGREES);
                    if (Math.abs(pod.getPosY(DistanceUnit.MM)) < 25 && Math.abs(pod.getPosX(DistanceUnit.MM)) < 25) {
                        move();
                    } else if (color.equals("Blue") && angle < 19) {
                        rotate();
                    } else if (color.equals("Red") && angle > -19) {
                        rotate();
                    } else {
                        state = states.SPIN_UP;
                        leftFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                    }
                } else {
                    System.out.println(waitTimer.seconds());
                }
                break;
            case SPIN_UP:
                if (feedTimer.seconds() > 1) {
                    state = (launcher.getVelocity() >= targetVelocity-20 && launcher.getVelocity() <= targetVelocity+20) ? states.LAUNCH : state;
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
                if(timesShot <= 4){
                    if (feedTimer.seconds() > 0.3) {
                        state = states.SPIN_UP;
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        timesShot += 1;
                        feedTimer.reset();
                    }
                } else {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    state = states.MOVE;
                }
                break;
            case MOVE:
                pod.update();
                if(Math.abs(pod.getPosY(DistanceUnit.MM)) < 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) < 200) {
                    move();
                }else if(Math.abs(pod.getPosX(DistanceUnit.MM)) < 150){
                    strafe();
                }else if (Math.abs(pod.getPosY(DistanceUnit.MM)) >= 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) >= 200) {
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    pod.update();
                    state = states.STOP_MOVE;
                }

                pod.update();

//                if (Math.abs(pod.getPosY(DistanceUnit.MM)) >= 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) >= 200) {
//                    leftFrontDrive.setPower(0);
//                    rightFrontDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    pod.update();
//                    state = states.STOP_MOVE;
//                }

                break;
            case STOP_MOVE:
                break;

        }
    }
    public abstract void move();
    public abstract void rotate();
    public abstract void unrotate();
    public abstract void reverse();
    public abstract void strafe();
    private enum states {
        NOT_READY,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        MOVE,
        STOP_MOVE
    }
}
