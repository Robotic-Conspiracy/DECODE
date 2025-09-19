package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Data {
    private Gamepad Gamepad1;
    private Gamepad Gamepad2;

    /**
     * Initialize all the data that can be passed to all the modules.
     *
     * @param gamepad1 The First Gamepad, Required to be passed from an OP mode
     * @param gamepad2 The Second Gamepad, Required to be passed from an OP mode
     * @author GoldStar184
     */
    public void init(Gamepad gamepad1, Gamepad gamepad2){
        Gamepad1 = gamepad1;
        Gamepad2 = gamepad2;
    }
    public Gamepad get(String data) throws IllegalArgumentException{
       switch (data){
           case "gamepad1":
               return Gamepad1;
           case "gamepad2":
               return Gamepad2;
           default:
               throw new IllegalArgumentException("data request is either non existent or not of Gamepad type");

       }
    }
}
