
package org.firstinspires.ftc.teamcode.autos.back;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public abstract class The_Fourth_Auto extends OpMode {
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

    ElapsedTime feedTimer = new ElapsedTime();


    public static double P = 203;
    public static double I = 1.001;
    public static double D = 0.0015;
    public static double F = 0.1;
    ///        launcher.setVelocityPIDFCoefficients(203, 1.001, 0.0015, 0.1);



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angleThing = hardwareMap.get(Servo.class, "left twideler");
        pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        pod.resetPosAndIMU();
        pod.setPosition(new Pose2D(DistanceUnit.MM,0,0,AngleUnit.RADIANS,0));
    }

    @Override
    public void loop() {
        pod.update();
        launcher.setVelocity(1480);
        telemetry.addData("Position", pod.getPosition());
        telemetry.addData("Velocity", launcher.getVelocity());
        launcher.setVelocityPIDFCoefficients(P,I,D,F);
        angleThing.setPosition(73/360.0);
        switch(state) {
            case NOT_READY:
                //leftFrontDrive.setPower(-1);
                //rightFrontDrive.setPower(1);
                //leftBackDrive.setPower(1);
                //rightBackDrive.setPower(-1);

//        pod.update();
//        if (Math.abs(pod.getPosY()) >= 75 || Math.abs(pod.getPosX()) >= 75) {
//          leftFrontDrive.setPower(0);
//          rightFrontDrive.setPower(0);
//          leftBackDrive.setPower(0);
//          rightBackDrive.setPower(0);
                state = states.SPIN_UP;
                //}

                break;
            case SPIN_UP:
                if (feedTimer.seconds() > 1) {
                    state = (launcher.getVelocity() >= 1480-20 && launcher.getVelocity() <= 1480+20) ? states.LAUNCH : state;
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
                    if (feedTimer.seconds() > 0.2) {
                        state = states.SPIN_UP;
                        leftFeeder.setPower(0);
                        rightFeeder.setPower(0);
                        timesShot += 1;
                    }
                } else {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    state = states.MOVE;
                }
                break;
            case MOVE:
                move();

                pod.update();
                telemetry.addData("Position", pod.getPosition());
                if (Math.abs(pod.getPosY(DistanceUnit.MM)) >= 150 || Math.abs(pod.getPosX(DistanceUnit.MM)) >= 150) {
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    pod.update();
                    state = states.STOP_MOVE;
                }

                break;
            case STOP_MOVE:
                break;

        }

        telemetry.addData("state", state);

    }
    abstract void move();
    private enum states {
        NOT_READY,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        MOVE,
        STOP_MOVE
    }
}

