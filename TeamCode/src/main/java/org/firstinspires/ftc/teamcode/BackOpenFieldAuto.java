package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class BackOpenFieldAuto extends OpMode {
  protected DcMotor leftFrontDrive;
  protected DcMotor rightFrontDrive;
  protected DcMotor leftBackDrive;
  protected DcMotor rightBackDrive;
  protected Servo angleThing;
  private DcMotorEx launcher;
  private CRServo leftFeeder;
  private CRServo rightFeeder;
  private GoBildaPinpointDriver pod;
  private states state = states.NOT_READY;
  private int timesShot = 0;

  private ElapsedTime feedTimer = new ElapsedTime();
  private ElapsedTime waitTimer = new ElapsedTime();
  protected String color = "None";

  private double targetVelocity = 1720;

  private AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
  private AprilTagProcessor aprilTagProcessor;
  private VisionPortal portal;
  private AprilTagDetection targetDetection = null;


  @Override
  public void init(){
    leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
    leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
    rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
    leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
    rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    angleThing = hardwareMap.get(Servo.class, "left_twideler");
    pod = hardwareMap.getAll(GoBildaPinpointDriver.class).get(0);

    leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);


    leftFrontDrive.setZeroPowerBehavior(BRAKE);
    rightFrontDrive.setZeroPowerBehavior(BRAKE);
    leftBackDrive.setZeroPowerBehavior(BRAKE);
    rightBackDrive.setZeroPowerBehavior(BRAKE);

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

    pod.resetPosAndIMU();

  }
  @Override
  public void start() {
    waitTimer.reset();
    System.out.println("reset wait timer");
    while(portal.getCameraState() != VisionPortal.CameraState.STREAMING){
      telemetry.addLine("Camera is not streaming");
      telemetry.update();
    }
    ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
      exposureControl.setMode(ExposureControl.Mode.Manual);
    }
    exposureControl.setExposure((long)8, TimeUnit.MILLISECONDS);
  }
  @Override
  public void loop() {
    launcher.setVelocity(targetVelocity);
    launcher.setVelocityPIDFCoefficients(203, 1.001, 0.0015, 0.1);
    angleThing.setPosition(56/360.0);
    pod.update();
    telemetry.addData("velocity ", launcher.getVelocity());
    telemetry.addData("position x", pod.getPosX(DistanceUnit.MM));
    telemetry.addData("position y", pod.getPosY(DistanceUnit.MM));
//        telemetry.addData("front left wheel", leftFrontDrive.getPower());
//        telemetry.addData("front right wheel", rightFrontDrive.getPower());
//        telemetry.addData("back left wheel", leftBackDrive.getPower());
//        telemetry.addData("back right wheel", rightBackDrive.getPower());

    //ToDo: finish setting up the state machine
    if(portal.getCameraState() == VisionPortal.CameraState.STREAMING) {


      switch(state) {
        case NOT_READY:
          if (waitTimer.seconds() >= 1) {
            pod.update();
            double angle = pod.getHeading(AngleUnit.DEGREES);
            telemetry.addData("podY", pod.getPosY(DistanceUnit.MM));
            telemetry.addData("podX", pod.getPosX(DistanceUnit.MM));
            telemetry.addData("angle", angle);

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            AprilTagDetection detectionRed = null;
            AprilTagDetection detectionBlue = null;
            double detectionpattern = 0;
            if (Math.abs(pod.getPosY(DistanceUnit.MM)) < 25 && Math.abs(pod.getPosX(DistanceUnit.MM)) < 25 ) {
              move();
            }else if (detections.isEmpty()){
              rotate();
            }else{

              if (!detections.isEmpty()) {
                for (AprilTagDetection Detection : detections) {
                  telemetry.addData("detecting id", Detection.id);
                  if (Detection.id == 24) {
                    detectionRed = Detection;
                  }
                  if (Detection.id == 20) {
                    detectionBlue = Detection;
                  }
                  if (Detection.id == 21) {
                    detectionpattern = 2;
                  }
                  if (Detection.id == 22) {
                    detectionpattern = 1;
                  }
                  if (Detection.id == 23) {
                    detectionpattern = 0;
                  }
                }


                if (color.equals("Blue") && detectionBlue != null) {
                  telemetry.addData("offset angle",detectionBlue.ftcPose.z);
                  if (detectionBlue.ftcPose.z > 0.5) {
                    drive(0, 0, Range.clip(detectionBlue.ftcPose.z * -0.05, -0.15, 0.15));// was -0.15
                  } else if (detectionBlue.ftcPose.z < -0.5) {
                    //Drive(0, 0, 1*(Math.abs(targetDetection.ftcPose.z)/100));// was 0.5
                    drive(0, 0, Range.clip(detectionBlue.ftcPose.z * -0.05, -0.15, 0.15));
                    //1*|offset|/15
                  } else {
                    if (detectionpattern > 0){
                      targetVelocity = 540;
                      angleThing.setPosition(62/360.0);
                      drive(0, 0, 0);
                      state = states.SPIN_UP;
                      detectionpattern -= 1;
                    }
                    if (detectionpattern <= 0){
                      targetVelocity = 1660;
                      angleThing.setPosition(56/360.0);
                      state = states.SPIN_UP;
                      drive(0, 0, 0);
                    }

                  }
                  telemetry.update();
                } else if (color.equals("Red") && detectionRed != null) {
                  telemetry.addData("offset angle",detectionRed.ftcPose.z);
                  if (detectionRed.ftcPose.z > 0.5) {
                    drive(0, 0, Range.clip(detectionRed.ftcPose.z * -0.05, -0.15, 0.15));// was -0.15
                  } else if (detectionRed.ftcPose.z < -0.5) {
                    //Drive(0, 0, 1*(Math.abs(targetDetection.ftcPose.z)/100));// was 0.5
                    drive(0, 0, Range.clip(detectionRed.ftcPose.z * -0.05, -0.15, 0.15));
                    //1*|offset|/15
                  } else {
                    state = states.SPIN_UP;
                    drive(0, 0, 0);
                  }
                } else {
                  rotate();
                }
              }
            }

          } else {
            System.out.println(waitTimer.seconds());
          }
          break;
        case SPIN_UP:
          if (feedTimer.seconds() > 3) {
            state = (launcher.getVelocity() >= targetVelocity - 20 && launcher.getVelocity() <= targetVelocity + 20) ? states.LAUNCH : state;
          }

          break;
        case LAUNCH:
          leftFeeder.setPower(1);
          rightFeeder.setPower(1);
          feedTimer.reset();
          state = states.LAUNCHING;
          break;
        case LAUNCHING:
          telemetry.addData("Feed time", feedTimer.seconds());
          if (timesShot <= 6) {
            if (feedTimer.seconds() > 0.3) {
              state = states.SPIN_UP;
              leftFeeder.setPower(0);
              rightFeeder.setPower(0);
              timesShot += 1;
              feedTimer.reset();
            }
          } else {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            state = states.MOVE;
          }
          break;
        case MOVE:
          //                move();
          //
          //                pod.update();
          //                telemetry.addData("Position", pod.getPosition());
          //                if (Math.abs(pod.getPosY(DistanceUnit.MM)) >= 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) >= 200) {
          //                    leftFrontDrive.setPower(0);
          //                    rightFrontDrive.setPower(0);
          //                    leftBackDrive.setPower(0);
          //                    rightBackDrive.setPower(0);
          //                    pod.update();
          //                    state = states.STOP_MOVE;
          //                }
          pod.update();
          if (Math.abs(pod.getPosY(DistanceUnit.MM)) < 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) < 200) {
            move();
          } else if (Math.abs(pod.getPosX(DistanceUnit.MM)) < 150) {
            strafe();
          } else if (Math.abs(pod.getPosY(DistanceUnit.MM)) >= 200 && Math.abs(pod.getPosX(DistanceUnit.MM)) >= 200) {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            pod.update();
            state = states.STOP_MOVE;
          }

          pod.update();

          break;
        case STOP_MOVE:
          break;
      }
    }
  }
  public abstract void move();
  public abstract void rotate();
  public abstract void strafe();

  public void drive(double forward, double strafe, double rotate) {


    double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

    leftFrontDrive.setPower((forward - strafe - rotate)/denominator);
    leftBackDrive.setPower((forward + strafe - rotate)/denominator);
    rightFrontDrive.setPower((forward + strafe + rotate)/denominator);
    rightBackDrive.setPower((forward - strafe + rotate)/denominator);
  }

  private enum states {
    NOT_READY,
    SPIN_UP,
    LAUNCH,
    LAUNCHING,
    MOVE,
    STOP_MOVE
  }
}
