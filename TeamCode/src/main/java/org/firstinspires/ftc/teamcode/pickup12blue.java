package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous coded by visualizer", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public double Wait3;
        public PathChain Path2;
        public PathChain Path4;
        public PathChain Path5;
        public double Wait7;
        public PathChain Path6;
        public PathChain Path8;
        public PathChain Path9;
        public double Wait10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public double Wait14;
        public PathChain Path15;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.000, 8.000), new Pose(57.812, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            Wait3 = 1000;

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.812, 15.882), new Pose(52.729, 35.576))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.729, 35.576), new Pose(16.729, 35.365))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.729, 35.365), new Pose(58.024, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait7 = 1000;

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.024, 15.882), new Pose(52.518, 59.929))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.518, 59.929), new Pose(17.365, 59.718))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.365, 59.718), new Pose(58.024, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait10 = 1000;

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.024, 15.882), new Pose(50.824, 84.071))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.824, 84.071), new Pose(17.365, 83.859))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.365, 83.859), new Pose(59.294, 84.282))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .setReversed()
                    .build();

            Wait14 = 1000;

            Path15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.506, 70.518), new Pose(25.200, 70.941))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}