package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Drive")
public class AutoDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorStorage;
    private DcMotor motorShoot;
    private Servo servoRotateBall;
    private Servo servoLoadBall;
    private Servo servoYaw;
    private Servo servoPitch;

    private NormalizedColorSensor colorSensorBall;
    private NormalizedColorSensor colorSensorBallRotation;

    private Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEAR_REDUCTION = 19.2;
    private static final double WHEEL_DIAMETER_MM = 40;
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);

    private static final double DRIVE_SPEED_DEADZONE = 0.05;
    private static final double DRIVE_SPEED_EXPO = 0.3;
    private static final double DRIVE_SPEED_RC_RATE = 0.7;
    private static final double DRIVE_SPEED_SUPER_RATE = 0.0;
    private static final double DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    private static final double TURN_SPEED_DEADZONE = 0.05;
    private static final double TURN_SPEED_EXPO = 0.5;
    private static final double TURN_SPEED_RC_RATE = 0.25;
    private static final double TURN_SPEED_SUPER_RATE = 0.3;
    private static final double TURN_SPEED_MAX_SENSITIVITY = 0.7;

    private static final boolean WEB_CAM_MANUAL_EXPOSURE = true;
    private static final int WEB_CAM_EXPOSURE_MS = 6;
    private static final int WEB_CAM_GAIN = 250;

    private static final float COLOR_SENSOR_GAIN = 0;

    // TODO: change to actual value
    private static final double BALL_DISTANCE_THRESHOLD_MM = 50.0;
    private static final double BALL_COLOR_DISTANCE_THRESHOLD = 0.4;
    private static final List<float[]> BALL_PURPLE_SAMPLES = Arrays.asList(
            new float[]{290, 0.7f, 0.5f},
            new float[]{300, 0.8f, 0.6f},
            new float[]{310, 0.75f, 0.55f},
            new float[]{295, 0.65f, 0.45f}
    );
    private static final List<float[]> BALL_GREEN_SAMPLES = Arrays.asList(
            new float[]{110, 0.8f, 0.7f},
            new float[]{120, 0.85f, 0.75f},
            new float[]{130, 0.8f, 0.7f},
            new float[]{115, 0.75f, 0.65f}
    );

    private static final double BALL_ROTATOR_TAG_DISTANCE_THRESHOLD_MM = 50.0;
    private static final double BALL_ROTATOR_TAG_COLOR_DISTANCE_THRESHOLD = 0.4;
    private static final List<float[]> BALL_ROTATOR_TAG_1_SAMPLES = Arrays.asList(
    );
    private static final List<float[]> BALL_ROTATOR_TAG_2_SAMPLES = Arrays.asList(
    );
    private static final List<float[]> BALL_ROTATOR_TAG_3_SAMPLES = Arrays.asList(
    );

    private static final double SERVO_ROTATE_BALL_SPEED = 0.5;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorRight = hardwareMap.get(DcMotor.class, "motor2");
        motorStorage = hardwareMap.get(DcMotor.class, "motor3");
        motorShoot = hardwareMap.get(DcMotor.class, "motor4");
        servoRotateBall = hardwareMap.get(Servo.class, "servo1");
        servoLoadBall = hardwareMap.get(Servo.class, "servo2");
        servoYaw = hardwareMap.get(Servo.class, "servo3");
        servoPitch = hardwareMap.get(Servo.class, "servo4");
        colorSensorBall = hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensorBallRotation = hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");

        telemetry.addLine("[Init] hardwareMap initialized");
        telemetry.update();

        initAprilTag();

        telemetry.addLine("[Init] AprilTag initialized");
        telemetry.update();

        if (WEB_CAM_MANUAL_EXPOSURE) {
            setManualExposure(WEB_CAM_EXPOSURE_MS, WEB_CAM_GAIN);
            telemetry.addData("Web Cam Exposure", "%d (ms)", WEB_CAM_EXPOSURE_MS);
            telemetry.addData("Web Cam Gain", "%d", WEB_CAM_GAIN);
            telemetry.update();
        }

        colorSensorBall.setGain(COLOR_SENSOR_GAIN);
        colorSensorBallRotation.setGain(COLOR_SENSOR_GAIN);

        telemetry.addLine(String.format(Locale.KOREA, "ColorSensor gain set to %f", COLOR_SENSOR_GAIN));

        if (colorSensorBall instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorBall).enableLight(true);
            telemetry.addLine("ColorSensor light enabled");
        }
        if (colorSensorBallRotation instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorBallRotation).enableLight(true);
            telemetry.addLine("ColorSensor light enabled");
        }
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStorage.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShoot.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
            telemetryAprilTag();

            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }

    private void encoderDrive(MotorAction left, MotorAction right, double timeout) {
        if (opModeIsActive()) {
            int newLeftTarget = motorLeft.getCurrentPosition() + (int)(left.target * COUNTS_PER_MM);
            int newRightTarget = motorRight.getCurrentPosition() + (int)(right.target * COUNTS_PER_MM);

            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            motorLeft.setPower(left.speed);
            motorRight.setPower(right.speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {
                telemetry.addData("Running to", " %7d | %7d",
                        newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " %7d | %7d",
                        motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
                telemetry.update();
            }

            motorLeft.setPower(0);
            motorRight.setPower(0);

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.KOREA, "\n==== (ID %d) %s", detection.id, detection.metadata.name));

                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format(Locale.KOREA, "XYZ %6.1f %6.1f %6.1f  (mm)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format(Locale.KOREA, "PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format(Locale.KOREA, "\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.KOREA, "Center %6.0f %6.0f  (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    private void detectTargetAprilTag(int desired_tag_id) {
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((desired_tag_id < 0) || (detection.id == desired_tag_id)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f mm", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f deg", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f deg", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("Not Found", "Desired tag not found");
        }
        telemetry.update();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }

            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void telemetryColor() {
        NormalizedRGBA colorsBall = colorSensorBall.getNormalizedColors();
        NormalizedRGBA colorsBallRotation = colorSensorBallRotation.getNormalizedColors();

        float[] hsvValuesBall = new float[3];
        float[] hsvValuesBallRotation = new float[3];
        Color.colorToHSV(colorsBall.toColor(), hsvValuesBall);
        Color.colorToHSV(colorsBallRotation.toColor(), hsvValuesBallRotation);

        double distanceBall = 0;
        double distanceBallRotation = 0;

        if (colorSensorBall instanceof DistanceSensor) {
            distanceBall = ((DistanceSensor)colorSensorBall).getDistance(DistanceUnit.MM);
        }
        if (colorSensorBallRotation instanceof DistanceSensor) {
            distanceBallRotation = ((DistanceSensor)colorSensorBallRotation).getDistance(DistanceUnit.MM);
        }

        telemetry.addLine("Color Ball")
                .addData("Red", "%.3f", colorsBall.red)
                .addData("Green", "%.3f", colorsBall.green)
                .addData("Blue", "%.3f", colorsBall.blue)
                .addData("Hue", "%.3f", hsvValuesBall[0])
                .addData("Saturation", "%.3f", hsvValuesBall[1])
                .addData("Value" ,"%.3f", hsvValuesBall[2])
                .addData("Alpha", "%.3f", colorsBall.alpha)
                .addData("Distance (mm)", "%.3f", distanceBall);

        telemetry.addLine("Color Ball Rotation")
                .addData("Red", "%.3f", colorsBallRotation.red)
                .addData("Green", "%.3f", colorsBallRotation.green)
                .addData("Blue", "%.3f", colorsBallRotation.blue)
                .addData("Hue", "%.3f", hsvValuesBallRotation[0])
                .addData("Saturation", "%.3f", hsvValuesBallRotation[1])
                .addData("Value" ,"%.3f", hsvValuesBallRotation[2])
                .addData("Alpha", "%.3f", colorsBallRotation.alpha)
                .addData("Distance (mm)", "%.3f", distanceBallRotation);
    }

    private double hsvDistance(float[] hsv1, float[] hsv2) {
        float hueDiff = Math.abs(hsv1[0] - hsv2[0]);
        if (hueDiff > 180) {
            hueDiff = 360 - hueDiff;
        }

        return Math.sqrt(Math.pow(hueDiff / 180.0, 2) + Math.pow(hsv1[1] - hsv2[1], 2) + Math.pow(hsv1[2] - hsv2[2], 2));
    }

    private double calculateColorDistance(float[] inputHsv, List<float[]> samples) {
        return samples.stream()
                .mapToDouble(sample -> hsvDistance(inputHsv, sample))
                .average()
                .orElse(Double.MAX_VALUE);
    }

    private DetectedColor detectBallColor(NormalizedRGBA colors, double distance) {
        if (distance > BALL_DISTANCE_THRESHOLD_MM) {
            return DetectedColor.NONE;
        }

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        double purpleColorDistance = calculateColorDistance(hsvValues, BALL_PURPLE_SAMPLES);
        double greenColorDistance = calculateColorDistance(hsvValues, BALL_GREEN_SAMPLES);

        double minSimilarity = Math.min(purpleColorDistance, greenColorDistance);

        if (minSimilarity > BALL_COLOR_DISTANCE_THRESHOLD) {
            return DetectedColor.NONE;
        }

        if (purpleColorDistance < greenColorDistance) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.GREEN;
        }
    }

    private DetectedColor detectBallRotatorTagColor(NormalizedRGBA colors, double distance) {
        if (distance > BALL_ROTATOR_TAG_DISTANCE_THRESHOLD_MM) {
            return DetectedColor.NONE;
        }

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        double tagColorDistance1 = calculateColorDistance(hsvValues, BALL_ROTATOR_TAG_1_SAMPLES);
        double tagColorDistance2 = calculateColorDistance(hsvValues, BALL_ROTATOR_TAG_2_SAMPLES);
        double tagColorDistance3 = calculateColorDistance(hsvValues, BALL_ROTATOR_TAG_3_SAMPLES);

        double minSimilarity = Math.min(Math.min(tagColorDistance1, tagColorDistance2), tagColorDistance3);

        if (minSimilarity > BALL_ROTATOR_TAG_COLOR_DISTANCE_THRESHOLD) {
            return DetectedColor.NONE;
        }

        if (tagColorDistance1 <= tagColorDistance2 && tagColorDistance1 <= tagColorDistance3) {
            return DetectedColor.TAG1;
        } else if (tagColorDistance2 <= tagColorDistance1 && tagColorDistance2 <= tagColorDistance3) {
            return DetectedColor.TAG2;
        } else {
            return DetectedColor.TAG3;
        }
    }

    private DetectedColor getStorageColor() {
        NormalizedRGBA colors = colorSensorBall.getNormalizedColors();

        double distance = 0;
        if (colorSensorBall instanceof DistanceSensor) {
            distance = ((DistanceSensor)colorSensorBall).getDistance(DistanceUnit.MM);
        }

        return detectBallColor(colors, distance);
    }
}
