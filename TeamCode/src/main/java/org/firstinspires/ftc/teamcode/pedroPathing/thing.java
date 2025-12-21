package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class thing extends OpMode {
    public static Follower follower;
    @IgnoreConfigurable
    static PoseHistory poseHistory;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private final Pose startPose = new Pose(63.573330647946776, 8.043551310945293, Math.toRadians(90));
    private final Pose endPose = new Pose(72, 72, Math.toRadians(141));
    private PathChain path;
    public static void draw() {
        Drawing.drawDebug(follower);
    }
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    @Override
    public void init() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(startPose);

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();

    }

    @Override
    public void loop() {
        follower.update();
        draw();



    }
    @Override
    public void start() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
            .build();
        follower.followPath(path);
    }

}
