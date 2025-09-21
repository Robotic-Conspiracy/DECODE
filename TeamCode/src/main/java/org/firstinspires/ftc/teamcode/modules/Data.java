package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Data {
    private Gamepad Gamepad1;
    private Gamepad Gamepad2;
    private HardwareMap hardwareMap;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    /**
     * Initialize all the data that can be passed to all the modules.
     * The Drive motors should be passed in the order of left front, right front, left back, right back
     *
     * @param gamepad1 The First Gamepad, Required to be passed from an OP mode
     * @param gamepad2 The Second Gamepad, Required to be passed from an OP mode
     * @param hardwareMap The Hardware Map, Required to be passed from an OP mode
     * @param leftFrontDrive The Left Front Drive Motor, Required to be passed from an OP mode
     * @author GoldStar184
     */
    public void init(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive){
        Gamepad1 = gamepad1;
        Gamepad2 = gamepad2;
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;


        this.hardwareMap = hardwareMap;
    }
    public Gamepad getGamepad(int index) throws IllegalArgumentException{
       switch (index){
           case 1:
               return Gamepad1;
           case 2:
               return Gamepad2;
           default:
               throw new IllegalArgumentException("Gamepad index must be 1 or 2");

       }
    }
    public HardwareMap getHardwareMap(){
        return hardwareMap;
    }
    public DcMotor getDrive(String side1, String side2) throws IllegalArgumentException{

        if (side1.equals("left")){
            if(side2.equals("front")){
                return leftFrontDrive;
            }
            else if(side2.equals("back")){
                return leftBackDrive;
            }
        }
        else if (side1.equals("right")){
            if(side2.equals("front")){
                return rightFrontDrive;
            }
            else if(side2.equals("back")){
                return rightBackDrive;
            }
        }

        throw new IllegalArgumentException("side1 must be left or right, side2 must be front or back");


    }
}
