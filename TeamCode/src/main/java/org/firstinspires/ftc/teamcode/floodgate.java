package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "FloodgateCurrentTest", group = "Sensor")
public class floodgate extends LinearOpMode {

    // Declare an AnalogInput object
    AnalogInput floodgateCurrent;

    @Override
    public void runOpMode() {

        // Initialize the AnalogInput object from the hardware map
        floodgateCurrent = hardwareMap.get(AnalogInput.class, "floodgateCurrent");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Read the raw voltage from the sensor (in Volts)
            double voltage = floodgateCurrent.getVoltage();

            // The goBILDA documentation (and forum posts) indicate the output is 0-3.3V
            // and often imply a direct correlation to current, but the specific
            // conversion factor (Amps per Volt) is needed.
            
            // A common range is 0-40A for 0-3.3V.
            // Assuming a linear relationship and a max of 40A at 3.3V:
            double amps = (voltage / 3.3) * 40.0;
            // *Note: This 40.0A max is an estimate based on the current limit and should be verified with the official goBILDA documentation for the precise scaling factor.*

            // Add the current data to telemetry
            telemetry.addData("Current (Amps)", "%.2f A", amps);
            telemetry.addData("Voltage (Sensor)", "%.2f V", voltage);
            telemetry.update();
        }
    }
}
