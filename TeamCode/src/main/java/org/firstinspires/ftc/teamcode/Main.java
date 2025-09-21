package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.Data;
import org.firstinspires.ftc.teamcode.modules.RobotModule;

import java.util.List;

@TeleOp(name = "Bolts and no Screws")
public class Main extends OpMode {
    public Data data = new Data();
    public List<RobotModule> modules;

    @Override
    public void init() {
        //TODO: add all modules here!
        for (RobotModule module:
             modules) {
            module.init(data); //Adds the data class to each module.
        }

    }

    @Override
    public void loop() {
    //TODO: Actual code here!
    }

}
