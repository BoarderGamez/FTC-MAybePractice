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

@TeleOp(name = "Werkend")
public class donttouch extends LinearOpMode {

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
  
  private static final double TARGET_DISTANCE_CM = 160.0;
  private static final double TARGET_X_OFFSET_CM = 20.0;
  private static final double TARGET_YAW = 0.0;
  
  private static final double DRIVE_GAIN = 0.015;
  private static final double STRAFE_GAIN = 0.015;
  private static final double TURN_GAIN = 0.02;
  
  private static final double MAX_DRIVE = 0.6;
  private static final double MAX_STRAFE = 0.6;
  private static final double MAX_TURN = 0.5;
  
  private static final double DISTANCE_TOLERANCE_CM = 5.0;
  private static final double X_TOLERANCE_CM = 2.5;
  private static final double YAW_TOLERANCE = 3.0;

  private static final double FEEDER_REST_VOLTAGE = 2.2;
  private static final double FEEDER_MAX_EXTEND_VOLTAGE = 1.5;

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
  private boolean firingSequence = false;
  private boolean waitingForFeederReturn = false;
  private boolean flywheelOn = false;
  private boolean leftTriggerPressed = false;
  private boolean recalibrated = false;
  private boolean backTrack =false;


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

      if (!safeMode) {
        handleGearShift();
        handleSpindexerCalibration();
        handleIntake();
        handleFlywheel();
        handleFeeder();
      }
      if (gamepad1.b){
        ballCount +=1;
        while (gamepad1.b){}
      }
      if (gamepad1.x){
        ballCount -=1;
        while (gamepad1.x) {
          
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
      double[] trackingOutput = getTagTracking();
      if (trackingOutput != null) {
        forward = trackingOutput[0];
        strafe = trackingOutput[1];
        turn = trackingOutput[2];
        telemetry.addData("Tag Lock", "LOCKED");
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
      intake.setPower(0);
      return;
    }

    switch (ballPosition) {
      case "inside":
        intake.setPower(0);
        if (spindexerReady && isFeederAtRest()) {
          detectBallColor();
          if (!detectedColor.equals("none") && ballCount < 3) {
            spindexerReady = false;
            spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
            ballCount++;
            detectedColor = "none";
          }
        }
        break;
      case "close":
        if (ballCount < 3) {
          intake.setPower(1);
        } else {
          intake.setPower(0);
        }
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
    if (gamepad1.left_trigger > 0.5) {
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
        spindexerReady = false;
        spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
        ballCount--;
        firingSequence = false;
      }
    }

      if (gamepad1.right_trigger > 0.2 && spindexerReady && !firingSequence && ballCount > 0 && ((DcMotorEx)flywheel2).getVelocity()>=1100) {
        feeder.setPosition(0.6);
        firingSequence = true;
      } else if (!firingSequence) {
        feeder.setPosition(0.06);
      }
  }

  private boolean isFeederAtRest() {
    return pot.getVoltage() > FEEDER_REST_VOLTAGE;
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

    if (Math.abs(rangeError) > DISTANCE_TOLERANCE_CM) {
      drivePower = -rangeError * DRIVE_GAIN;
      drivePower = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, drivePower));
    }

    if (Math.abs(xError) > X_TOLERANCE_CM) {
      strafePower = -xError * STRAFE_GAIN;
      strafePower = Math.max(-MAX_STRAFE, Math.min(MAX_STRAFE, strafePower));
    }

    if (Math.abs(yawError) > YAW_TOLERANCE) {
      turnPower = -yawError * TURN_GAIN;
      turnPower = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnPower));
    }

    telemetry.addData("Range", "%.1f cm (err: %.1f)", rangeCm, rangeError);
    telemetry.addData("X Offset", "%.1f cm (err: %.1f)", xOffsetCm, xError);
    telemetry.addData("Yaw", "%.1f° (err: %.1f)", yaw, yawError);

    return new double[]{drivePower, strafePower, turnPower};
  }

  private void updateTelemetry() {
    double currentAmp = ((DcMotorEx) flywheel1).getCurrent(CurrentUnit.AMPS) +
                        ((DcMotorEx) flywheel2).getCurrent(CurrentUnit.AMPS);
    shooterMaxAmp = Math.max(shooterMaxAmp, currentAmp);

    telemetry.addData("Gearbox", "%.1f", gearbox);
    telemetry.addData("Balls", ballCount);
    telemetry.addData("Auto Intake", autoOn ? "ON" : "OFF");
    telemetry.addData("Spindexer", spindexerReady ? "READY" : "BUSY");
    telemetry.addData("Feeder Pot", "%.2fV (rest: %s)", pot.getVoltage(), isFeederAtRest() ? "YES" : "NO");
    telemetry.addData("Max Shooter Amps", "%.2f", shooterMaxAmp);
    telemetry.addData("Shooter speed1", ((DcMotorEx)flywheel1).getVelocity());
    telemetry.addData("Shooter speed2", ((DcMotorEx)flywheel2).getVelocity());
    
    telemetry.update();
  }
}
