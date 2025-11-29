package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class autoAlignmentTest extends OpMode {
    private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private final int TARGET_ID = 24;//blue is 20

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private AprilTagDetection targetDetection = null;


    //21 - 23 is the obolysk
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //aprilTagProcessorBuilder.setLensIntrinsics(811.775, 811.775, 364.462, 208.771);
        aprilTagProcessor = aprilTagProcessorBuilder.build();

        aprilTagProcessor.setDecimation(3);
//        portal = new VisionPortal.Builder()
//                .setCamera(BuiltinCameraDirection.BACK)
//                .addProcessor(aprilTagProcessor)
//                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    @Override
    public void loop() {
        if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long)10, TimeUnit.MILLISECONDS);
        }

        targetDetection = null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("detections", detections);
        for (AprilTagDetection detection : detections) {

            if(detection.id == TARGET_ID) {
//                telemetry.addData("bearing offset", detection.ftcPose.bearing);
//                telemetry.addData("yaw offset", detection.ftcPose.yaw);
//                telemetry.addData("range offset", detection.ftcPose.range);
//                telemetry.addData("x offset", detection.ftcPose.x);
//                telemetry.addData("y offset", detection.ftcPose.y);
                telemetry.addData("z offset", detection.ftcPose.z);//- rotate right  + rotate left
                targetDetection = detection;

//                telemetry.addData("elevation offset", detection.ftcPose.elevation);
//                telemetry.addData("pitch offset", detection.ftcPose.pitch);
//                telemetry.addData("roll offset", detection.ftcPose.roll);

                break;
            } else {
                Drive(0,0,0);
            }
        }
        if(targetDetection != null) {
            if(gamepad1.x){
                if(targetDetection.ftcPose.z > 1){
                    Drive(0, 0, Range.clip(targetDetection.ftcPose.z*-0.05, -0.15, 0.15));// was -0.15
                } else if (targetDetection.ftcPose.z < -1) {
                    //Drive(0, 0, 1*(Math.abs(targetDetection.ftcPose.z)/100));// was 0.5
                    Drive(0, 0, Range.clip(targetDetection.ftcPose.z*-0.05, -0.15, 0.15));
                    //1*|offset|/15
                } else {
                    Drive(0,0,0);
                }
            } else {
                Drive(0,0,0);
            }
        } else {
            Drive(0,0,0);
        }

        telemetry.addData("camera state",portal.getCameraState());
        telemetry.update();
    }


    private void Drive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        frontLeftMotor.setPower((forward - strafe - rotate)/denominator);
        backLeftMotor.setPower((forward + strafe - rotate)/denominator);
        frontRightMotor.setPower((forward + strafe + rotate)/denominator);
        backRightMotor.setPower((forward - strafe + rotate)/denominator);

    }

}
