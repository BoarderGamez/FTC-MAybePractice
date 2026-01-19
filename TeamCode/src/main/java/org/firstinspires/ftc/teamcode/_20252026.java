package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "_20252026 (Blocks to Java)")
public class _20252026 extends LinearOpMode {

  private DcMotor flywheel1;
  private DcMotor flywheel2;
  private DcMotor spindexer;
  private Servo feeder;
  private DcMotor intake;
  private DcMotor FL;
  private DcMotor FR;
  private DcMotor RL;
  private DcMotor RR;
  private ColorSensor color;
  private TouchSensor hallsensor;
  private DistanceSensor dist;
  private DistanceSensor color_DistanceSensor;

  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;

  private static final int TARGET_TAG_ID = 20;
  private static final double AUTO_AIM_GAIN = 0.1;
  private static final double AUTO_AIM_MAX_TURN = 0.8;

  private boolean autoOn = true;
  private double gearbox = 0.4;
  private boolean spindexerCalibrating = true;
  private boolean spindexerReady = false;
  private String detectedColor = "none";
  private double ballDistance = 100.0;
  private int ballCount = 0;
  private double shooterMaxAmp = 0;

  private boolean calibrationPhase1Complete = false;
  private boolean gearShiftPressed = false;
  private boolean aButtonPressed = false;
  private boolean yButtonPressed = false;
  private boolean autoAimEnabled = false;
  private boolean safeMode = false;

  @Override
  public void runOpMode() {
    initHardware();
    initAprilTag();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    safeMode = gamepad1.ps;
    feeder.setPosition(0);
    sleep(2000);

    ElapsedTime loopTimer = new ElapsedTime();

    while (opModeIsActive()) {
      loopTimer.reset();

      handleDriving();

      if (!safeMode) {
        handleGearShift();
        handleSpindexerCalibration();
        handleIntake();
        handleFlywheel();
        handleFeeder();
      }

      updateTelemetry();
    }

    if (visionPortal != null) {
      visionPortal.close();
    }
  }

  private void initHardware() {
    flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
    flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
    spindexer = hardwareMap.get(DcMotor.class, "spindexer");
    feeder = hardwareMap.get(Servo.class, "feeder");
    intake = hardwareMap.get(DcMotor.class, "intake");
    FL = hardwareMap.get(DcMotor.class, "FL");
    FR = hardwareMap.get(DcMotor.class, "FR");
    RL = hardwareMap.get(DcMotor.class, "RL");
    RR = hardwareMap.get(DcMotor.class, "RR");
    color = hardwareMap.get(ColorSensor.class, "color");
    hallsensor = hardwareMap.get(TouchSensor.class, "hall sensor");
    dist = hardwareMap.get(DistanceSensor.class, "dist");
    color_DistanceSensor = hardwareMap.get(DistanceSensor.class, "color");

    flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel1.setDirection(DcMotor.Direction.REVERSE);
    spindexer.setDirection(DcMotor.Direction.REVERSE);
    feeder.setDirection(Servo.Direction.REVERSE);
    intake.setDirection(DcMotor.Direction.REVERSE);

    FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FL.setDirection(DcMotor.Direction.REVERSE);
    FR.setDirection(DcMotor.Direction.REVERSE);
    RL.setDirection(DcMotor.Direction.FORWARD);
    RR.setDirection(DcMotor.Direction.FORWARD);

    ((NormalizedColorSensor) color).setGain(24);
  }

  private void initAprilTag() {
    aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .build();

    visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
  }

  private void handleDriving() {
    double forward = gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;

    if (gamepad1.y && !yButtonPressed) {
      autoAimEnabled = !autoAimEnabled;
      yButtonPressed = true;
    } else if (!gamepad1.y) {
      yButtonPressed = false;
    }

    if (autoAimEnabled) {
      double autoAimTurn = getAutoAimTurn();
      if (autoAimTurn != 0) {
        turn = autoAimTurn;
        telemetry.addData("Auto-Aim", "AIMING");
      } else {
        telemetry.addData("Auto-Aim", "NO TARGET");
      }
    } else {
      telemetry.addData("Auto-Aim", "OFF");
    }

    FL.setPower((forward + (strafe - turn)) * gearbox);
    RL.setPower((forward - (strafe + turn)) * gearbox);
    FR.setPower((forward + strafe + turn) * gearbox);
    RR.setPower((forward - (strafe - turn)) * gearbox);
  }

  private void handleGearShift() {
    if (!gearShiftPressed) {
      if (gamepad1.right_bumper) {
        gearbox += 0.1;
        gearShiftPressed = true;
      } else if (gamepad1.left_bumper) {
        gearbox -= 0.1;
        gearShiftPressed = true;
      }
    } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
      gearShiftPressed = false;
    }
    gearbox = Math.max(0.2, Math.min(1.0, gearbox));
  }

  private void handleSpindexerCalibration() {
    if (gamepad1.ps) {
      spindexerCalibrating = true;
      calibrationPhase1Complete = false;
    }

    if (spindexerCalibrating) {
      spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      if (!calibrationPhase1Complete) {
        if (hallsensor.isPressed()) {
          spindexer.setPower(-0.2);
        } else {
          spindexer.setPower(0);
          calibrationPhase1Complete = true;
        }
      } else {
        if (hallsensor.isPressed()) {
          spindexer.setPower(0);
          spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          spindexer.setTargetPosition(-20);
          spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) spindexer).setTargetPositionTolerance(10);
          spindexer.setPower(1);
          spindexerCalibrating = false;
        } else {
          spindexer.setPower(0.1);
        }
      }
    }

    if (!spindexer.isBusy()) {
      spindexerReady = true;
    }
  }

  private void handleIntake() {
    if (gamepad1.a && !aButtonPressed) {
      autoOn = !autoOn;
      aButtonPressed = true;
    } else if (!gamepad1.a) {
      aButtonPressed = false;
    }

    ballDistance = dist.getDistance(DistanceUnit.CM);
    double colorDistance = color_DistanceSensor.getDistance(DistanceUnit.CM);

    String ballPosition;
    if (ballDistance < 6 || colorDistance < 8) {
      ballPosition = "inside";
    } else if (ballDistance < 36) {
      ballPosition = "close";
    } else {
      ballPosition = "far";
    }

    if (!autoOn) {
      return;
    }

    switch (ballPosition) {
      case "inside":
        intake.setPower(0);
        if (spindexerReady) {
          detectBallColor();
          if (!detectedColor.equals("none") && ballCount < 3) {
            spindexerReady = false;
            spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
            ballCount++;
          }
        }
        break;
      case "close":
        intake.setPower(1);
        break;
      case "far":
      default:
        intake.setPower(0);
        break;
    }
  }

  private void detectBallColor() {
    int green = color.green();
    int blue = color.blue();
    int alpha = color.alpha();

    if (alpha == 0) {
      detectedColor = "none";
      return;
    }

    double greenRatio = (double) green / alpha;
    double blueRatio = (double) blue / alpha;

    if (greenRatio > 1.2) {
      detectedColor = "green";
    } else if (blueRatio > 1.2) {
      detectedColor = "purple";
    } else {
      detectedColor = "none";
    }
  }

  private void handleFlywheel() {
    if (gamepad1.left_trigger > 0.1) {
      flywheel1.setPower(0.8);
      flywheel2.setPower(0.8);
    } else {
      flywheel1.setPower(0);
      flywheel2.setPower(0);
    }
  }

  private void handleFeeder() {
    if (gamepad1.right_trigger > 0.1 && spindexerReady) {
      feeder.setPosition(0.4);
    } else {
      feeder.setPosition(0.05);
    }
  }

  private double getAutoAimTurn() {
    List<AprilTagDetection> detections = aprilTag.getDetections();

    AprilTagDetection goalTag = null;
    for (AprilTagDetection detection : detections) {
      if (detection.id == TARGET_TAG_ID) {
        goalTag = detection;
        break;
      }
    }

    if (goalTag == null || goalTag.ftcPose == null) {
      return 0;
    }

    double xOffset = goalTag.ftcPose.x;

    telemetry.addData("Tag ID", goalTag.id);
    telemetry.addData("X Offset", "%.1f in", xOffset);
    telemetry.addData("Range", "%.1f in", goalTag.ftcPose.range);

    if (Math.abs(xOffset) < 1.0) {
      telemetry.addData("Aligned", "YES");
      return 0;
    }

    double turnPower = xOffset * AUTO_AIM_GAIN;
    turnPower = Math.max(-AUTO_AIM_MAX_TURN, Math.min(AUTO_AIM_MAX_TURN, turnPower));

    return turnPower;
  }

  private void updateTelemetry() {
    double currentAmp = ((DcMotorEx) flywheel1).getCurrent(CurrentUnit.AMPS) +
                        ((DcMotorEx) flywheel2).getCurrent(CurrentUnit.AMPS);
    shooterMaxAmp = Math.max(shooterMaxAmp, currentAmp);

    telemetry.addData("Gearbox", "%.1f", gearbox);
    telemetry.addData("Balls", ballCount);
    telemetry.addData("Auto Intake", autoOn ? "ON" : "OFF");
    telemetry.addData("Spindexer", spindexerReady ? "READY" : "BUSY");
    telemetry.addData("Max Shooter Amps", "%.2f", shooterMaxAmp);
    telemetry.update();
  }
}
