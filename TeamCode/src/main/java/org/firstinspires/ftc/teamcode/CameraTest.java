package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class CameraTest extends LinearOpMode {
    final boolean USING_WEBCAM = false;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;
    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;

    boolean lastX;
    int frameCount;
    long capReqTime;


    @Override
    public void runOpMode() throws InterruptedException {
        VisionPortal portal;
        aprilTagProcessorBuilder.setLensIntrinsics(811.775, 811.775, 364.462, 208.771);
        aprilTagProcessorBuilder.setCameraPose(new Position(DistanceUnit.MM, 285, 291, 45, System.currentTimeMillis()), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, System.currentTimeMillis()));
        aprilTagProcessorBuilder.setDrawTagOutline(true);
        aprilTagProcessor = aprilTagProcessorBuilder.build();
        if (USING_WEBCAM)
        {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .build();
        }
        else
        {
            portal = new VisionPortal.Builder()
                    .setCamera(INTERNAL_CAM_DIR)
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .addProcessor(aprilTagProcessor)
                    .build();
        }
        while (!isStopRequested())
        {
            if (portal.getCameraState() == VisionPortal.CameraState.STREAMING){
                if (aprilTagProcessor.getDetections().size() != 0){
                    telemetry.addData("detections",
                            GetAllDetectionIDs(aprilTagProcessor.getDetections()).toString()
                            );
                } else {
                    telemetry.addData("detections", "none");
                }

            }
        }
        telemetry.update();
    }

    private ArrayList<Integer> GetAllDetectionIDs (ArrayList<AprilTagDetection> detections){
        ArrayList<Integer> detection_ids = new ArrayList<>();
        for(int i = 0; i < detections.size(); i++){
            detection_ids.add(detections.get(i).id);
        }
        return detection_ids;
    }
}
