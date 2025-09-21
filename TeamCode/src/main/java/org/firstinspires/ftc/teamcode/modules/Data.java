package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Data {
    private Gamepad Gamepad1;
    private Gamepad Gamepad2;
    private HardwareMap hardwareMap;

    /**
     * Initialize all the data that can be passed to all the modules.
     *
     * The Drive motors are assumed to be all named
     *
     * @param gamepad1 The First Gamepad, Required to be passed from an OP mode
     * @param gamepad2 The Second Gamepad, Required to be passed from an OP mode
     * @param hardwareMap The Hardware Map, Required to be passed from an OP mode
     * @author GoldStar184
     */
    public void init(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap){
        Gamepad1 = gamepad1;
        Gamepad2 = gamepad2;

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
}
