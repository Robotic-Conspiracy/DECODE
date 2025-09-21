package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.modules.Data;
import org.firstinspires.ftc.teamcode.modules.DriveModule;
import org.firstinspires.ftc.teamcode.modules.RobotModule;

import java.util.List;

@TeleOp(name = "Bolts and no Screws")
public class Main extends OpMode {
    public Data data = new Data();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    public List<RobotModule> modules;
    public void init() {


        //While we don't use these here, we are modifying them here they and can be gotten from the data class later.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //Initialize the data class
        data.init(gamepad1, gamepad2, hardwareMap, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

        //set behavior to break so faster slowdown for better control over the robot.
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



        //TODO: add all modules here!
        modules.add(new DriveModule());
        for (RobotModule module:
             modules) {
            module.init(data); //Adds the data class to each module.
        }

    }

    @Override
    public void loop() {
        for (RobotModule module:
             modules) {
            module.run();
        }
    }

}
