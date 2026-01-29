package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

@TeleOp(name = "wip")
public class wipcode extends LinearOpMode {

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
  private AnalogInput pot;

  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;

  private static final int TARGET_TAG_ID = 24;
  private static final double INCHES_TO_CM = 2.54;
  
  private static final double TARGET_DISTANCE_CM = 230.0;
  private static final double TARGET_X_OFFSET_CM = 0.0;
  private static final double TARGET_YAW = 0.0;
  private static final double ROTATE_MODE_X_OFFSET_CM = 10.0;
  private static final double ROTATE_MODE_DISTANCE_CM = 230.0;
  
  private static final double DRIVE_GAIN = 0.015;
  private static final double STRAFE_GAIN = 0.015;
  private static final double TURN_GAIN = 0.02;
  
  private static final double MAX_DRIVE = 0.6;
  private static final double MAX_STRAFE = 0.6;
  private static final double MAX_TURN = 0.5;
  
  private static final double DISTANCE_TOLERANCE_CM = 5.0;
  private static final double X_TOLERANCE_CM = 2.5;
  private static final double YAW_TOLERANCE = 3.0;
  private static final double FULL_ALIGN_DISTANCE_CM = 200.0;

  private static final double FEEDER_REST_VOLTAGE = 2.2;
  private static final double FEEDER_MAX_EXTEND_VOLTAGE = 1.5;

  private boolean autoOn = true;
  private static final boolean AUTO_SHOOTER_ENABLED = false;
  private double gearbox = 0.4;
  private boolean spindexerCalibrating = true;
  private boolean spindexerReady = false;
  private String detectedColor = "none";
  private double ballDistance = 100.0;
  private int ballCount = 1;
  private double shooterMaxAmp = 0;

  private String[] slotColors = {"none", "none", "none"};
  
  private static final String[][] SHOOTING_ORDERS = {
    {"purple", "purple", "green"},  // PPG - Tag 23
    {"purple", "green", "purple"},  // PGP - Tag 22
    {"green", "purple", "purple"}   // GPP - Tag 21
  };
  private static final int[] ORDER_TAG_IDS = {23, 22, 21};
  private int currentOrderIndex = 0;
  private String[] currentShootingOrder = SHOOTING_ORDERS[0];
  private int shootingOrderIndex = 0;
  private int storageCount = 0;
  private boolean p2RightBumperPressed = false;
  private boolean p2LeftBumperPressed = false;
  private boolean p2APressed = false;
  private boolean sortingRotation = false;
  private int sortingTargetSlot = -1;
  private boolean pendingColorUpdate = false;
  private String pendingColor = "none";

  private boolean calibrationPhase1Complete = false;
  private boolean gearShiftPressed = false;
  private boolean aButtonPressed = false;
  private boolean yButtonPressed = false;
  private boolean autoAimEnabled = false;
  private boolean rotateToTagEnabled = false;
  private boolean bButtonPressed = false;
  private boolean safeMode = false;
  private boolean firingSequence = false;
  private boolean waitingForFeederReturn = false;
  private boolean flywheelOn = false;
  private boolean leftTriggerPressed = false;
  private boolean autoShootingCycle = false;
  private int prevBallCount = 0;
  private boolean ballBeingProcessed = false;
  private boolean recalibrated = false;
  private boolean backTrack = false;
  private boolean intakeModeActive = false;
  private boolean intakeModeUsed = false;


  @Override
  public void runOpMode() {
    initHardware();
    initAprilTag();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    safeMode = gamepad1.ps;
    feeder.setPosition(0.06);

    ElapsedTime loopTimer = new ElapsedTime();

    while (opModeIsActive()) {
      loopTimer.reset();

      handleDriving();
      checkOrderTags();

      if (!safeMode) {
        handleGearShift();
        handleSpindexerCalibration();
        handleIntake();
        handleFlywheel();
        handleFeeder();
      }
      if (gamepad1.dpad_right){
        ballCount +=1;
        while (gamepad1.dpad_right){}
      }
      if (gamepad1.dpad_left){
        ballCount -=1;
        if (ballCount < 1) {
          ballCount = 1;
        }
        while (gamepad1.dpad_left) {
          
        }
      }

      if (ballCount ==0 && !recalibrated){
        spindexerCalibrating = true;
        calibrationPhase1Complete = false;
        recalibrated = true;

      }
      if (ballCount !=0 || ballCount<3){
        recalibrated = false;
      }

      if (slotColors[0].equals("none") && slotColors[1].equals("none") && slotColors[2].equals("none")) {
        ballCount = 1;
      }

      if (gamepad2.right_bumper && !p2RightBumperPressed) {
        storageCount++;
        p2RightBumperPressed = true;
      } else if (!gamepad2.right_bumper) {
        p2RightBumperPressed = false;
      }

      if (gamepad2.left_bumper && !p2LeftBumperPressed) {
        storageCount--;
        if (storageCount < 0) {
          storageCount = 0;
        }
        p2LeftBumperPressed = true;
      } else if (!gamepad2.left_bumper) {
        p2LeftBumperPressed = false;
      }

      if (gamepad2.a && !p2APressed) {
        storageCount = 0;
        p2APressed = true;
      } else if (!gamepad2.a) {
        p2APressed = false;
      }

      shootingOrderIndex = storageCount % 3;

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
    pot = hardwareMap.get(AnalogInput.class, "pot");

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
    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    if (gamepad1.b && !bButtonPressed) {
      rotateToTagEnabled = !rotateToTagEnabled;
      bButtonPressed = true;
    } else if (!gamepad1.b) {
      bButtonPressed = false;
    }

    if (autoAimEnabled) {
      double[] trackingOutput = getTagTracking();
      if (trackingOutput != null) {
        forward = trackingOutput[0];
        strafe = trackingOutput[1];
        turn = trackingOutput[2];
        telemetry.addData("Tag Lock", "LOCKED");
      } else {
        telemetry.addData("Tag Lock", "NO TARGET");
      }
    } else if (rotateToTagEnabled) {
      double[] trackingOutput = getRotateToTagTracking();
      if (trackingOutput != null) {
        turn = trackingOutput[0];
        strafe = trackingOutput[1];
        telemetry.addData("Tag Lock", "ROTATE + DISTANCE");
      } else {
        telemetry.addData("Tag Lock", "NO TARGET");
      }
    } else {
      telemetry.addData("Tag Lock", "OFF");
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
      slotColors = new String[]{"none", "none", "none"};
      ballCount = 1;
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
      ballBeingProcessed = false;
      
      if (pendingColorUpdate) {
        rotateSlotColorsAnticlockwise();
        slotColors[1] = pendingColor;
        pendingColorUpdate = false;
        pendingColor = "none";
      }
    }

    if (ballCount == 0 && prevBallCount > 0 && spindexerReady && !spindexerCalibrating) {
      spindexerCalibrating = true;
      calibrationPhase1Complete = false;
    }

    prevBallCount = ballCount;
  }

  private void handleIntake() {
    if (gamepad1.a && !aButtonPressed) {
      autoOn = !autoOn;
      aButtonPressed = true;
    } else if (!gamepad1.a) {
      aButtonPressed = false;
    }

    if (!gamepad1.x) {
      intakeModeUsed = false;
    }

    if (gamepad1.x && !intakeModeUsed && spindexerReady && !sortingRotation) {
      int emptySlot = findSlotWithColor("none");
      if (emptySlot == 2) {
        intakeModeActive = true;
        spindexerReady = false;
        spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
        rotateSlotColorsAnticlockwise();
      } else if (emptySlot == 1) {
        intakeModeActive = true;
        spindexerReady = false;
        spindexer.setTargetPosition(spindexer.getTargetPosition() - 380);
        rotateSlotColorsClockwise();
      } else if (emptySlot == 0) {
        intakeModeActive = true;
      }
      intakeModeUsed = true;
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

    if (intakeModeActive && ballPosition.equals("inside") && spindexerReady) {
      intakeModeActive = false;
    }

    if (!autoOn) {
      intake.setPower(0);
      return;
    }

    if (sortingRotation || !spindexerReady) {
      intake.setPower(0);
      return;
    }

    if (!slotColors[0].equals("none")) {
      intake.setPower(0);
      return;
    }

    switch (ballPosition) {
      case "inside":
        intake.setPower(0);
        if (!ballBeingProcessed && spindexerReady && isFeederAtRest() && !spindexerCalibrating) {
          detectBallColor();
          if (!detectedColor.equals("none") && ballCount < 4) {
            pendingColor = detectedColor;
            pendingColorUpdate = true;
            spindexerReady = false;
            spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
            ballCount++;
            detectedColor = "none";
            ballBeingProcessed = true;
          }
        }
        break;
      case "close":
        if (ballCount < 4) {
          intake.setPower(1);
        } else {
          intake.setPower(0);
        }
        break;
      case "far":
      default:
        if (spindexerReady) {
          ballBeingProcessed = false;
        }
        intake.setPower(0);
        break;
    }
  }

  private void rotateSlotColorsAnticlockwise() {
    String temp = slotColors[2];
    slotColors[2] = slotColors[1];
    slotColors[1] = slotColors[0];
    slotColors[0] = temp;
  }

  private void rotateSlotColorsClockwise() {
    String temp = slotColors[0];
    slotColors[0] = slotColors[1];
    slotColors[1] = slotColors[2];
    slotColors[2] = temp;
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
    if (AUTO_SHOOTER_ENABLED) {
      if (ballCount >= 4) {
        autoShootingCycle = true;
      }
      if (ballCount <= 1) {
        autoShootingCycle = false;
      }
    }

    if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
      flywheelOn = !flywheelOn;
      leftTriggerPressed = true;
      if (!flywheelOn) {
        autoAimEnabled = false;
        rotateToTagEnabled = false;
      }
    } else if (gamepad1.left_trigger < 0.3) {
      leftTriggerPressed = false;
    }

    if (autoShootingCycle || flywheelOn) {
      ((DcMotorEx) flywheel1).setVelocity(1200);
      ((DcMotorEx) flywheel2).setVelocity(1200);
    } else {  
      ((DcMotorEx) flywheel1).setVelocity(0);
      ((DcMotorEx) flywheel2).setVelocity(0);
    }
  }

  private void handleFeeder() {
    double voltage = pot.getVoltage();

    if (voltage < FEEDER_MAX_EXTEND_VOLTAGE) {
      feeder.setPosition(0.06);
      waitingForFeederReturn = true;
      return;
    }

    if (waitingForFeederReturn && voltage > FEEDER_REST_VOLTAGE) {
      waitingForFeederReturn = false;
      if (firingSequence && ballCount > 0) {
        slotColors[2] = "none";
        spindexerReady = false;
        spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
        rotateSlotColorsAnticlockwise();
        ballCount--;
        if (ballCount < 1) {
          ballCount = 1;
        }
        storageCount++;
        firingSequence = false;
      }
    }

    if (sortingRotation && spindexerReady) {
      sortingRotation = false;
    }

    boolean flywheelReady = ((DcMotorEx)flywheel2).getVelocity() >= 1170 && ((DcMotorEx)flywheel2).getVelocity() <= 1230;

    if (gamepad1.right_trigger > 0.2 && spindexerReady && !firingSequence && !sortingRotation && !intakeModeActive && ballCount > 0) {
      
      String neededColor = currentShootingOrder[shootingOrderIndex];
      
      if (slotColors[2].equals(neededColor)) {
        if (flywheelReady) {
          feeder.setPosition(0.6);
          firingSequence = true;
        }
      } else if (flywheelReady) {
        int correctSlot = findSlotWithColor(neededColor);
        
        if (correctSlot == 1) {
          sortingRotation = true;
          spindexerReady = false;
          spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
          rotateSlotColorsAnticlockwise();
        } else if (correctSlot == 0) {
          sortingRotation = true;
          spindexerReady = false;
          spindexer.setTargetPosition(spindexer.getTargetPosition() - 380);
          rotateSlotColorsClockwise();
        } else if (correctSlot == -1 && !slotColors[2].equals("none")) {
          feeder.setPosition(0.6);
          firingSequence = true;
        }
      }
    } else if (!firingSequence) {
      feeder.setPosition(0.06);
    }
  }

  private int findSlotWithColor(String color) {
    for (int i = 0; i < slotColors.length; i++) {
      if (slotColors[i].equals(color)) {
        return i;
      }
    }
    return -1;
  }

  private boolean isFeederAtRest() {
    return pot.getVoltage() > FEEDER_REST_VOLTAGE;
  }

  private void checkOrderTags() {
    List<AprilTagDetection> detections = aprilTag.getDetections();
    
    for (AprilTagDetection detection : detections) {
      for (int i = 0; i < ORDER_TAG_IDS.length; i++) {
        if (detection.id == ORDER_TAG_IDS[i] && currentOrderIndex != i) {
          currentOrderIndex = i;
          currentShootingOrder = SHOOTING_ORDERS[i];
          shootingOrderIndex = 0;
          break;
        }
      }
    }
  }

  private String getOrderName() {
    switch (currentOrderIndex) {
      case 0: return "PPG";
      case 1: return "PGP";
      case 2: return "GPP";
      default: return "???";
    }
  }

  private double[] getTagTracking() {
    List<AprilTagDetection> detections = aprilTag.getDetections();

    AprilTagDetection tag = null;
    for (AprilTagDetection detection : detections) {
      if (detection.id == TARGET_TAG_ID) {
        tag = detection;
        break;
      }
    }

    if (tag == null || tag.ftcPose == null) {
      return null;
    }

    double rangeCm = tag.ftcPose.range * INCHES_TO_CM;
    double xOffsetCm = tag.ftcPose.x * INCHES_TO_CM;
    double yaw = tag.ftcPose.yaw;

    double rangeError = rangeCm - TARGET_DISTANCE_CM;
    double xError = xOffsetCm - TARGET_X_OFFSET_CM;
    double yawError = yaw - TARGET_YAW;

    double drivePower = 0;
    double strafePower = 0;
    double turnPower = 0;

    boolean closeEnough = rangeCm < FULL_ALIGN_DISTANCE_CM;

    if (Math.abs(xError) > X_TOLERANCE_CM) {
      drivePower = -xError * DRIVE_GAIN;
      drivePower = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, drivePower));
    }

    if (Math.abs(yawError) > YAW_TOLERANCE) {
      turnPower = -yawError * TURN_GAIN;
      turnPower = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnPower));
    }

    if (Math.abs(rangeError) > DISTANCE_TOLERANCE_CM) {
      strafePower = -rangeError * STRAFE_GAIN;
      strafePower = Math.max(-MAX_STRAFE, Math.min(MAX_STRAFE, strafePower));
    }

    telemetry.addData("Range", "%.1f cm (err: %.1f)", rangeCm, rangeError);
    telemetry.addData("X Offset", "%.1f cm (err: %.1f)", xOffsetCm, xError);
    telemetry.addData("Yaw", "%.1f° (err: %.1f)", yaw, yawError);

    return new double[]{drivePower, strafePower, turnPower};
  }

  private double[] getRotateToTagTracking() {
    List<AprilTagDetection> detections = aprilTag.getDetections();

    AprilTagDetection tag = null;
    for (AprilTagDetection detection : detections) {
      if (detection.id == TARGET_TAG_ID) {
        tag = detection;
        break;
      }
    }

    if (tag == null || tag.ftcPose == null) {
      return null;
    }

    double rangeCm = tag.ftcPose.range * INCHES_TO_CM;
    double xOffsetCm = tag.ftcPose.x * INCHES_TO_CM;
    double rangeError = rangeCm - ROTATE_MODE_DISTANCE_CM;
    double xError = xOffsetCm - ROTATE_MODE_X_OFFSET_CM;

    double turnPower = 0;
    double strafePower = 0;

    if (Math.abs(xError) > X_TOLERANCE_CM) {
      turnPower = xError * 0.04;
      turnPower = Math.max(-0.8, Math.min(0.8, turnPower));
    }

    if (Math.abs(rangeError) > DISTANCE_TOLERANCE_CM) {
      strafePower = -rangeError * 0.03;
      strafePower = Math.max(-0.9, Math.min(0.9, strafePower));
    }

    telemetry.addData("Range", "%.1f cm (err: %.1f)", rangeCm, rangeError);
    telemetry.addData("X Offset", "%.1f cm (err: %.1f)", xOffsetCm, xError);

    return new double[]{turnPower, strafePower};
  }

  private void updateTelemetry() {
    double currentAmp = ((DcMotorEx) flywheel1).getCurrent(CurrentUnit.AMPS) +
                        ((DcMotorEx) flywheel2).getCurrent(CurrentUnit.AMPS);
    shooterMaxAmp = Math.max(shooterMaxAmp, currentAmp);

    telemetry.addData("=== STORAGE ===", storageCount);
    telemetry.addData("Order Pattern", "%s (Tag %d)", getOrderName(), ORDER_TAG_IDS[currentOrderIndex]);
    telemetry.addData("Next Shot", "%d/3 - Need: %s", shootingOrderIndex + 1, currentShootingOrder[shootingOrderIndex]);
    telemetry.addData("Gearbox", "%.1f", gearbox);
    telemetry.addData("Balls in Robot", ballCount);
    telemetry.addData("Slots", "[1:%s] [2:%s] [3:%s]", slotColors[0], slotColors[1], slotColors[2]);
    telemetry.addData("Auto Intake", autoOn ? "ON" : "OFF");
    telemetry.addData("Spindexer", spindexerReady ? "READY" : (sortingRotation ? "SORTING" : "BUSY"));
    telemetry.addData("Intake Mode", intakeModeActive ? "ACTIVE" : "OFF");
    telemetry.addData("Feeder Pot", "%.2fV (rest: %s)", pot.getVoltage(), isFeederAtRest() ? "YES" : "NO");
    telemetry.addData("Max Shooter Amps", "%.2f", shooterMaxAmp);
    telemetry.addData("Shooter speed1", ((DcMotorEx)flywheel1).getVelocity());
    telemetry.addData("Shooter speed2", ((DcMotorEx)flywheel2).getVelocity());
    telemetry.addData("Detected Color", detectedColor);
    telemetry.addData("Ball Distance", "%.1f cm", ballDistance);
    
    telemetry.update();
  }
}
