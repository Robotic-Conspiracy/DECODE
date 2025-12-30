package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "the 12 bluest of blue balls of the holiest holly pantheist pantheon", group = "Autonomous")
@Configurable // Panels
public class pickup12blue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int nextPathState;
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

        public PathChain shootPreload;
        public double Wait3;
        public PathChain prePickup1;
        public PathChain pickup1;
        public PathChain shootPickup1;
        public double Wait7;
        public PathChain prePickup2;
        public PathChain pickUp2;
        public PathChain shootPickup2;
        public double Wait10;
        public PathChain prePickup3;
        public PathChain pickup3;
        public PathChain shootPickup3;
        public double Wait14;
        public PathChain Path15;


        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.000, 8.000), new Pose(57.812, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            Wait3 = 1000;

            prePickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.812, 15.882), new Pose(52.729, 35.576))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.729, 35.576), new Pose(16.729, 35.365))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootPickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.729, 35.365), new Pose(58.024, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait7 = 1000;

            prePickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58, 16), new Pose(52.518, 59.929))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            pickUp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.518, 59.929), new Pose(17.365, 59.718))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootPickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.365, 59.718), new Pose(58.024, 15.882))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            Wait10 = 1000;

            prePickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.024, 15.882), new Pose(50.824, 84.071))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.824, 84.071), new Pose(17.365, 83.859))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootPickup3 = follower
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
        switch (pathState) {
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootPreload);
                }
                if (!follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            case 7:
                break;
            case 8:
                break;
            case 9:
                break;
            case 10:
                break;
            case 11:
                break;
            case 12:
                break;
            case 13:
                break;
            case 14:
                break;
            case 15:
                break;
            case 100:
                // TODO use case 100 for aiming to launch
                pathState = 101;
                break;
            case 101:
                // TODO use case 101 for launch
                pathState = nextPathState;
                break;
            case 102:
                // TODO use case 102 as a pause to start the intake
                break;
        }
        return pathState;
    }
}