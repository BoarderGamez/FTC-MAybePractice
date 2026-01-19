package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Auto", group = "Autonomous")
public class AprilTagAuto extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor RL;
    private DcMotor RR;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("# AprilTags Detected", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== Tag ID %d (%s) ====", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f inches",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f degrees",
                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("Range: %.1f inches", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing: %.1f degrees", detection.ftcPose.bearing));

                    driveToTag(detection);
                } else {
                    telemetry.addLine(String.format("\n==== Unknown Tag ID %d ====", detection.id));
                }
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initHardware() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);
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

    private void driveToTag(AprilTagDetection detection) {
        double DESIRED_DISTANCE = 12.0;
        double SPEED_GAIN = 0.02;
        double STRAFE_GAIN = 0.015;
        double TURN_GAIN = 0.01;
        double MAX_SPEED = 0.5;

        double rangeError = detection.ftcPose.range - DESIRED_DISTANCE;
        double yawError = detection.ftcPose.yaw;
        double xError = detection.ftcPose.x;

        double drive = rangeError * SPEED_GAIN;
        double strafe = -xError * STRAFE_GAIN;
        double turn = -yawError * TURN_GAIN;

        drive = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, drive));
        strafe = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, strafe));
        turn = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, turn));

        if (Math.abs(rangeError) < 1.0 && Math.abs(yawError) < 2.0 && Math.abs(xError) < 1.0) {
            stopMotors();
            return;
        }

        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double rearLeftPower = drive - strafe + turn;
        double rearRightPower = drive + strafe - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower)));
        if (maxPower > MAX_SPEED) {
            frontLeftPower /= maxPower / MAX_SPEED;
            frontRightPower /= maxPower / MAX_SPEED;
            rearLeftPower /= maxPower / MAX_SPEED;
            rearRightPower /= maxPower / MAX_SPEED;
        }

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        RL.setPower(rearLeftPower);
        RR.setPower(rearRightPower);
    }

    private void stopMotors() {
        FL.setPower(0);
        FR.setPower(0);
        RL.setPower(0);
        RR.setPower(0);
    }
}
