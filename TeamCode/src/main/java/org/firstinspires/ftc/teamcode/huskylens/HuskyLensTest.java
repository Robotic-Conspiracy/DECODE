package org.firstinspires.ftc.teamcode.huskylens;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.List;

@Autonomous
public class HuskyLensTest extends LinearOpMode {
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime elapsedTime;
        HuskyLens.Block[] huskyLensBlocks;
        HuskyLens.Block huskyLensBlock;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        telemetry.addData(">> ", huskyLens.knock() ? "Press start to continue" : "Problem communicating with HuskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();
        elapsedTime = new ElapsedTime();
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                // Put loop blocks here.
                if (elapsedTime.seconds() >= 1) {
                    elapsedTime.reset();
                    huskyLensBlocks = huskyLens.blocks();
                    telemetry.addData("Block count", JavaUtil.listLength(huskyLensBlocks));
                    for (HuskyLens.Block myHuskyLensBlock_item : huskyLensBlocks) {
                        huskyLensBlock = myHuskyLensBlock_item;
                        telemetry.addData("Block", "id=" + huskyLensBlock.id + " size: " + huskyLensBlock.width + "x" + huskyLensBlock.height + " position: " + huskyLensBlock.x + "," + huskyLensBlock.y);
                    }
                    telemetry.update();
                }
            }
        }

    }
}
