package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * AutoPidFTuner
 *
 * Automatic velocity-PIDF tuner for an FTC DcMotorEx using the relay (ultimate gain) method.
 *
 * How to use:
 * - Put this file in TeamCode, update the motor name below to match your configuration.
 * - Run the OpMode. It will:
 *    1) Reset encoders and go RUN_WITHOUT_ENCODER (open-loop).
 *    2) Run a relay toggle around the chosen velocity setpoint.
 *    3) Measure oscillation period Pu and amplitude d.
 *    4) Compute Ku, Kp, Ki, Kd using Ziegler-Nichols.
 *    5) Compute kF = steadyPower / setpointVelocity (power per ticks/sec).
 *    6) Optionally apply the coefficients to the motor (RUN_USING_ENCODER) and print/save them.
 *
 * Safety: keep speeds/powers conservative during first runs.
 */
@TeleOp(name = "Auto PIDF Tuner", group = "tuning")
public class AutoPidFTuner extends LinearOpMode {

    // CONFIGURE THIS
    private final String MOTOR_NAME = "launcher"; // change to your motor config name
    private final double SETPOINT_VELOCITY_TPS = 1500.0; // target velocity in encoder ticks per second (pick representative speed)
    private final double RELAY_POWER = 0.45; // power applied by the relay (0..1). Tune to provoke oscillation.
    private final double MAX_TUNING_SECONDS = 20.0; // max time to attempt tuning
    private final int REQUIRED_TOGGLES = 12; // number of relay toggles to record (half-cycles); ~6 full cycles
    private final boolean APPLY_TO_MOTOR = true; // set true to write the computed PIDF to the motor's RUN_USING_ENCODER controller
    private final boolean SAVE_TO_FILE = true; // if true will attempt to save JSON to phone internal storage

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // Ensure encoder is reset and motor in open-loop for relay test
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addData("Tuner", "Starting relay oscillation test");
        telemetry.addData("Config", "Setpoint tps=%.1f, relayPower=%.2f", SETPOINT_VELOCITY_TPS, RELAY_POWER);
        telemetry.update();

        // Run the relay experiment
        RelayResult result = runRelayTest(motor, SETPOINT_VELOCITY_TPS, RELAY_POWER, MAX_TUNING_SECONDS, REQUIRED_TOGGLES);

        if (!result.success) {
            telemetry.addData("Result", "Failed to produce clear oscillation: %s", result.message);
            telemetry.update();
            motor.setPower(0.0);
            return;
        }

        double Pu = result.Pu;
        double d = result.amplitude; // half of peak-to-peak (d)
        double A = RELAY_POWER;

        // Prevent division by zero
        if (d <= 1e-6) {
            telemetry.addData("Error", "Measured amplitude too small (d=%.6f)", d);
            telemetry.update();
            motor.setPower(0.0);
            return;
        }

        // Ultimate gain
        double Ku = (4.0 * A) / (Math.PI * d);

        // Ziegler-Nichols PID (classic)
        double Kp = 0.6 * Ku;
        double Ki = 1.2 * Ku / Pu;       // Ki as "per second" integral gain (ZN formula)
        double Kd = 0.075 * Ku * Pu;

        // Simple feedforward: power per ticks/sec (verify and scale for your hub if needed)
        double kF = A / SETPOINT_VELOCITY_TPS;

        telemetry.addData("Pu (s)", "%.3f", Pu);
        telemetry.addData("d (ticks/sec)", "%.3f", d);
        telemetry.addData("Ku", "%.3f", Ku);
        telemetry.addData("Calculated PIDF", "Kp=%.5f Ki=%.5f Kd=%.5f kF=%.8f", Kp, Ki, Kd, kF);
        telemetry.update();

        // Optionally apply to the motor's RUN_USING_ENCODER velocity controller
        if (APPLY_TO_MOTOR) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients coeffs = new PIDFCoefficients(Kp, Ki, Kd, kF);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
            telemetry.addData("Applied", "Set motor RUN_USING_ENCODER PIDF");
            telemetry.update();
        }

        // Optionally save to a file on the phone for later copy/paste
        if (SAVE_TO_FILE) {
            try {
                String json = String.format(
                        "{ \"motor\":\"%s\", \"setpoint_tps\":%.3f, \"relay_power\":%.3f, \"Pu\":%.5f, \"d\":%.5f, \"Ku\":%.5f, \"Kp\":%.8f, \"Ki\":%.8f, \"Kd\":%.8f, \"kF\":%.10f }",
                        MOTOR_NAME, SETPOINT_VELOCITY_TPS, RELAY_POWER, Pu, d, Ku, Kp, Ki, Kd, kF
                );
                // Save to app-private storage /sdcard/FIRST/data/pidf_tuning.json if available
                FileOutputStream fos = openFileOutput("pidf_tuning.json", MODE_PRIVATE);
                OutputStreamWriter writer = new OutputStreamWriter(fos);
                writer.write(json);
                writer.close();
                fos.close();
                telemetry.addData("Saved", "pidf_tuning.json");
            } catch (Exception e) {
                telemetry.addData("SaveError", e.toString());
            }
            telemetry.update();
        }

        telemetry.addLine("Tuning complete. Press Stop to finish.");
        telemetry.update();

        // leave motor stopped
        motor.setPower(0.0);

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private static class RelayResult {
        boolean success;
        String message;
        double Pu;        // oscillation period (s)
        double amplitude; // half peak-to-peak (d)
    }

    /**
     * Run a relay (bang-bang) experiment around a velocity setpoint.
     *
     * Relay output is +relayPower when velocity < setpoint, -relayPower when velocity >= setpoint.
     * We record toggle times (changes in relay output) and measured velocities during the oscillation.
     *
     * Returns Pu (seconds) and amplitude d (half peak-to-peak in ticks/sec).
     */
    private RelayResult runRelayTest(DcMotorEx motor, double setpointTps, double relayPower, double timeoutSeconds, int requiredToggles) throws InterruptedException {
        RelayResult res = new RelayResult();
        res.success = false;

        // safety defaults
        long startTime = System.nanoTime();
        double lastOutput = 0.0;
        double output = relayPower; // initial output choice (we'll set based on initial velocity)
        List<Long> toggleTimesNs = new ArrayList<>();
        double globalMax = Double.NEGATIVE_INFINITY;
        double globalMin = Double.POSITIVE_INFINITY;

        // read a few initial velocity samples to choose initial sign
        for (int i = 0; i < 10 && !isStopRequested(); i++) {
            double v = motor.getVelocity();
            if (v > globalMax) globalMax = v;
            if (v < globalMin) globalMin = v;
            sleep(20);
        }

        double velocity = motor.getVelocity();
        // choose initial output so that velocity moves toward the setpoint polarity
        output = (velocity < setpointTps) ? relayPower : -relayPower;
        motor.setPower(output);
        lastOutput = output;
        toggleTimesNs.add(System.nanoTime());

        // Run loop collecting toggles
        while (!isStopRequested()) {
            velocity = motor.getVelocity();
            if (velocity > globalMax) globalMax = velocity;
            if (velocity < globalMin) globalMin = velocity;

            // decide output
            output = (velocity < setpointTps) ? relayPower : -relayPower;
            motor.setPower(output);

            // detect toggle
            if (Math.signum(output) != Math.signum(lastOutput)) {
                toggleTimesNs.add(System.nanoTime());
                lastOutput = output;
            }

            // stop when we have enough toggles
            if (toggleTimesNs.size() >= requiredToggles) break;

            // handle timeout
            double elapsed = (System.nanoTime() - startTime) / 1e9;
            if (elapsed > timeoutSeconds) {
                res.message = String.format("Timeout after %.1fs, toggles=%d", elapsed, toggleTimesNs.size());
                motor.setPower(0.0);
                return res;
            }

            sleep(10);
        }

        // stop motor
        motor.setPower(0.0);

        if (toggleTimesNs.size() < 4) {
            res.message = "Not enough toggles recorded";
            return res;
        }

        // compute average half-period (time between consecutive toggles)
        List<Double> halfPeriods = new ArrayList<>();
        for (int i = 1; i < toggleTimesNs.size(); i++) {
            double dt = (toggleTimesNs.get(i) - toggleTimesNs.get(i - 1)) / 1e9; // seconds
            halfPeriods.add(dt);
        }

        // average half period
        double halfPeriodAvg = 0.0;
        for (double d : halfPeriods) halfPeriodAvg += d;
        halfPeriodAvg /= halfPeriods.size();

        double Pu = 2.0 * halfPeriodAvg;

        // amplitude d as half of (globalMax - globalMin)
        double amplitude = (globalMax - globalMin) / 2.0;

        if (amplitude <= 1e-6) {
            res.message = String.format("Amplitude too small (max=%.3f min=%.3f)", globalMax, globalMin);
            return res;
        }

        res.success = true;
        res.Pu = Pu;
        res.amplitude = amplitude;
        res.message = "OK";
        return res;
    }
}
