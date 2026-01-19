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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

  double gearbox;
  boolean spindex_calibration_bizzy;
  boolean spindexer_located;
  String color3;
  double ball_distance;
  boolean auto_on_off;
  int ball_number;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ElapsedTime myElapsedTime;
    boolean servo_allowence;
    boolean safe_mode;
    float forward;
    float strafe;
    float turn;
    double shooter_max_amp;

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

    for (int count = 0; count < 1; count++) {
      gearbox = 0.4;
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
    waitForStart();
    spindexer_located = false;
    spindex_calibration_bizzy = true;
    myElapsedTime = new ElapsedTime();
    servo_allowence = false;
    ball_number = 0;
    auto_on_off = true;
    ball_distance = "far";
    if (opModeIsActive()) {
      if (gamepad1.ps) {
        safe_mode = true;
      } else {
        safe_mode = false;
      }
      feeder.setPosition(0);
      sleep(2000);
      while (opModeIsActive()) {
        if (opModeIsActive()) {
          forward = gamepad1.left_stick_y;
          strafe = gamepad1.left_stick_x;
          turn = gamepad1.right_stick_x;
          FL.setPower((forward + (strafe - turn)) * gearbox);
          RL.setPower((forward - (strafe + turn)) * gearbox);
          FR.setPower((forward + strafe + turn) * gearbox);
          RR.setPower((forward - (strafe - turn)) * gearbox);
        }
        if (safe_mode) {
        } else {
          gearshift();
          spindex_calibration();
          auto_out_intake();
          flywheel();
          launch_servo();
        }
        telemetry.addData("max shooter amp", Math.max(shooter_max_amp, (shooter_max_amp = ((DcMotorEx) flywheel1).getCurrent(CurrentUnit.AMPS) + ((DcMotorEx) flywheel2).getCurrent(CurrentUnit.AMPS))));
        myElapsedTime.reset();
        telemetry.addData("balls", ball_number);
        telemetry.addData("timer", myElapsedTime.seconds());
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void gearshift() {
    boolean gear_shift_active;

    if (!gear_shift_active) {
      if (gamepad1.right_bumper) {
        gear_shift_active = true;
        gearbox = gearbox + 0.1;
      } else if (gamepad1.left_bumper) {
        gear_shift_active = true;
        gearbox = gearbox - 0.1;
      }
    } else if (!(gamepad1.a || gamepad1.left_bumper || gamepad1.right_bumper)) {
      gear_shift_active = false;
    }
    gearbox = Math.min(Math.max(gearbox, 0.2), 1);
  }

  /**
   * Describe this function...
   */
  private void spindex_calibration() {
    boolean spind_1;

    if (gamepad1.ps) {
      spindex_calibration_bizzy = true;
      spind_1 = false;
    }
    if (spindex_calibration_bizzy) {
      spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      if (!spind_1) {
        if (hallsensor.isPressed()) {
          spindexer.setPower(-0.2);
        } else {
          spindexer.setPower(0);
          spind_1 = true;
        }
      } else {
        if (hallsensor.isPressed()) {
          spindexer.setPower(0);
          spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          spindexer.setTargetPosition(-20);
          spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) spindexer).setTargetPositionTolerance(10);
          spindexer.setPower(1);
          spindex_calibration_bizzy = false;
        } else {
          spindexer.setPower(0.1);
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void auto_out_intake() {
    boolean t1;
    String ball_position;

    if (!spindexer.isBusy()) {
      spindexer_located = true;
    }
    if (gamepad1.a) {
      if (!t1) {
        if (auto_on_off) {
          auto_on_off = false;
        } else {
          auto_on_off = true;
        }
      }
      t1 = true;
    } else {
      t1 = false;
    }
    ball_distance = dist.getDistance(DistanceUnit.CM);
    if (spindexer_located) {
      if (ball_distance < 6 || color_DistanceSensor.getDistance(DistanceUnit.CM) < 8) {
        ball_position = "inside";
      } else if (dist.getDistance(DistanceUnit.CM) < 36) {
        ball_position = "close";
      } else {
        ball_position = "far";
      }
    }
    if (auto_on_off) {
      if (ball_position.equals("inside")) {
        intake.setPower(0);
        if (spindexer_located) {
          color2();
          if (!color3.equals("none") && ball_number != 3) {
            spindexer_located = false;
            spindexer.setTargetPosition(spindexer.getTargetPosition() + 380);
            ball_number += ball_number == 3 ? -2 : 1;
          }
        }
      } else if (ball_position.equals("close")) {
        intake.setPower(1);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void color2() {
    String color_bal_1;
    String color_bal_2;
    String color_bal_3;

    if (color.green() / color.alpha() > 1.2) {
      color3 = "green";
      if (ball_number == 1) {
        color_bal_1 = "purple";
      } else if (ball_number == 2) {
        color_bal_2 = "purple";
      } else if (ball_number == 3) {
        color_bal_3 = "purple";
      }
    } else if (color.blue() / color.alpha() > 1.2) {
      color3 = "green";
      if (ball_number == 1) {
        color_bal_1 = "green";
      } else if (ball_number == 2) {
        color_bal_2 = "green";
      } else if (ball_number == 3) {
        color_bal_3 = "green";
      }
    } else {
      color3 = "none";
      if (ball_number == 1) {
        color_bal_1 = "none";
      } else if (ball_number == 2) {
        color_bal_2 = "none";
      } else if (ball_number == 3) {
        color_bal_3 = "none";
      }
    }
  }

  /**
   * Describe this function...
   */
  private void launch_servo() {
    boolean block;

    if (gamepad1.right_trigger != 0 && spindexer_located) {
      block = true;
      feeder.setPosition(0.4);
    } else {
      feeder.setPosition(0.05);
      block = false;
    }
  }

  /**
   * Describe this function...
   */
  private void flapwheels() {
    boolean flapwheels_bp;
    boolean flapwheels_active;

    if (gamepad1.b && !flapwheels_bp) {
      flapwheels_bp = true;
      if (flapwheels_active) {
        flapwheels_active = false;
        intake.setPower(0);
      } else if (!flapwheels_active) {
        flapwheels_active = true;
        intake.setPower(1);
      }
    } else if (!gamepad1.b) {
      flapwheels_bp = false;
    }
  }

  /**
   * Describe this function...
   */
  private void flywheel() {
    if (gamepad1.left_trigger != 0) {
      flywheel1.setPower(0.8);
      flywheel2.setPower(0.8);
    } else {
      flywheel1.setPower(0);
      flywheel2.setPower(0);
    }
  }
}