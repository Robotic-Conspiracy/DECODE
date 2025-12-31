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
import org.firstinspires.ftc.teamcode.autos.pedroPathing.Constants;

    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class back3main extends OpMode {

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

            public PathChain shootPreload;
            public double Wait2;
            public PathChain park;
            public Pose shootPreloadStart, shootPreloadEnd;
            public Pose parkStart, parkEnd;
            public  int headShootPreload1;
            public  int headShootPreload2;
            public int headpark1;
            public int headpark2;

            public Paths(Follower follower) {
                shootPreload = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(63.000, 8), new Pose(60, 11))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                        .build();

                Wait2 = 1;

                park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(59.718, 11.435), new Pose(36.000, 10.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))
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
}
