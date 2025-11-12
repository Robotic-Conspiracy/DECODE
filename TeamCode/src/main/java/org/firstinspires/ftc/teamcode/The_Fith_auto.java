package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class The_Fith_auto extends OpMode {
    protected DcMotor leftFrontDrive;
    protected DcMotor rightFrontDrive;
    protected DcMotor leftBackDrive;
    protected DcMotor rightBackDrive;
    private GoBildaPinpointDriver pod;

    @Override
    public void init(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        pod.resetPosAndIMU();
        pod.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.RADIANS,0));
    }

    @Override
    public void loop() {
        pod.update();
        if(Math.abs(pod.getPosX(DistanceUnit.MM)) < 200 && Math.abs(pod.getPosY(DistanceUnit.MM)) < 200){
            move();
        } else {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }
    public abstract void move();
}
