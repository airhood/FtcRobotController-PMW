package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "Tele Drive")
public class TeleDriveNoIMU extends LinearOpMode {

    private Alliance alliance;

    private boolean ESR = false;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private IMU imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorStorage;
    private DcMotor motorShoot;
    private CRServo servoRotateBall;
    private Servo servoLoadBall;
    private CRServo servoYaw;
    private Servo servoPitch;

    private NormalizedColorSensor colorSensorBall;

    private Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private DetectedColor ballRotatorTarget = DetectedColor.NONE;

    private BallLoadState ballLoadState = BallLoadState.IDLE;
    private ElapsedTime ballLoadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double motorShootPower = 0.7;

    private double kP = 0.02;
    private double kD = 0.005;
    private double deadband = 1.0;
    private double lastError = 0;
    private double lastTime = 0;
    private double maxPower = 0.5;
    private double minPower = -0.5;

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
    private static double CAM_Fx = 1385.31;
    private static double CAM_Fy = 1385.31;
    private static double CAM_Cx = 973.922;
    private static double CAM_Cy = 554.487;

    private static final float COLOR_SENSOR_GAIN = 0;

    // TODO: change to actual value
    private static final double BALL_DISTANCE_THRESHOLD_MM = 30.0;
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

    //private static final double SERVO_ROTATE_BALL_SPEED = -0.5;
    private static final double SERVO_ROTATE_BALL_SPEED = 0;

    private static final double SERVO_LOAD_BALL_IDLE_POSITION = 0.9;
    private static final double SERVO_LOAD_BALL_ACTIVE_POSITION = 0.35;
    private static final double LOAD_BALL_SERVO_LIMIT = 0.02;
    private static final long LOAD_BALL_LOADED_WAIT_MS = 700;

    private static final double SERVO_PITCH_SPEED = 0.005;
    private static final double SERVO_PITCH_MAX = 0.3;
    private static final double SERVO_PITCH_MIN = 0.07;

    //private static final double MOTOR_STORAGE_SPEED = 0.42;
    private static final double MOTOR_STORAGE_SPEED = 0.42;

    @Override
    public void runOpMode() {
        initialize();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !gamepad1.b) alliance = Alliance.BLUE;
            else if (!gamepad1.x && gamepad1.b) alliance = Alliance.RED;

        }

        waitForStart();

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStorage.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShoot.setDirection(DcMotorSimple.Direction.FORWARD);

        motorStorage.setPower(MOTOR_STORAGE_SPEED);

        while (opModeIsActive()) {
            tick();

            if (ESR) {
                telemetry.addData("ESR", "Triggered");
            }
            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }

    private void initialize() {
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorRight = hardwareMap.get(DcMotor.class, "motor2");
        motorStorage = hardwareMap.get(DcMotor.class, "motor3");
        motorShoot = hardwareMap.get(DcMotor.class, "motor4");
        servoRotateBall = hardwareMap.get(CRServo.class, "servo1");
        servoLoadBall = hardwareMap.get(Servo.class, "servo2");
        servoYaw = hardwareMap.get(CRServo.class, "servo3");
        servoPitch = hardwareMap.get(Servo.class, "servo4");
        colorSensorBall = hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");

        telemetry.addLine("[Init] hardwareMap initialized");
        telemetry.update();

        telemetry.addLine("[Init] IMU initialized");

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

        telemetry.addLine(String.format(Locale.KOREA, "ColorSensor gain set to %f", COLOR_SENSOR_GAIN));

        if (colorSensorBall instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorBall).enableLight(true);
            telemetry.addLine("ColorSensor light enabled");
        }
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void tick() {
        controllerKey();

        // emergency break
        if (!gamepad1.left_bumper) {
            boolean isBallRotationManualControl = ballRotationManualControl();

            telemetryAprilTag();

            moveWheel();

            if (!isBallRotationManualControl) {
                ballRotatorTick();
            }

            loadBallTick();
            ballShootAlignmentTick();
            shootTick();
        }
    }

    private void controllerKey() {
        if (gamepad1.startWasPressed()) {
            telemetry.addData("Status", "Emergency system reboot: re-initializing");
            telemetry.update();
            initialize();
            telemetry.addData("Status", "Emergency system reboot: initialize success");
            telemetry.update();
        }

        if (gamepad1.backWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad1.rightBumperWasPressed()) {
            shootBall();
        }

        if (gamepad1.y) {
            double nextPos = servoPitch.getPosition() + SERVO_PITCH_SPEED;
            if (nextPos > SERVO_PITCH_MAX) nextPos = SERVO_PITCH_MAX;
            servoPitch.setPosition(nextPos);
        }
        if (gamepad1.a) {
            double nextPos = servoPitch.getPosition() - SERVO_PITCH_SPEED;
            if (nextPos < SERVO_PITCH_MIN) nextPos = SERVO_PITCH_MIN;
            servoPitch.setPosition(nextPos);
        }

        if (gamepad1.dpad_down) {
            motorShoot.setPower(0);
        } else {
            motorShoot.setPower(MOTOR_STORAGE_SPEED);
        }
    }

    private boolean ballRotationManualControl() {
        if (!gamepad1.x && gamepad1.b) { // ball rotation default direction
            servoRotateBall.setPower(SERVO_ROTATE_BALL_SPEED);
            return true;
        } else if (gamepad1.x && !gamepad1.b) { // ball rotation opposite direction
            servoRotateBall.setPower(-SERVO_ROTATE_BALL_SPEED);
            return true;
        }
        return false;
    }

    private void moveWheel() {
        double leftX = processStickInput(gamepad1.left_stick_x, TURN_SPEED_DEADZONE, TURN_SPEED_EXPO, TURN_SPEED_RC_RATE, TURN_SPEED_SUPER_RATE, TURN_SPEED_MAX_SENSITIVITY);
        double rightY = processStickInput(gamepad1.right_stick_y, DRIVE_SPEED_DEADZONE, DRIVE_SPEED_EXPO, DRIVE_SPEED_RC_RATE, DRIVE_SPEED_SUPER_RATE, DRIVE_SPEED_MAX_SENSITIVITY);
        rightY *= -1;

        double left = rightY + leftX;
        double right = rightY - leftX;

        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));

        if (maxMagnitude > 1.0) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }

        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    public static double processStickInput(double value, double deadzone, double expo,
                                           double rcRate, double superRate, double maxSensitivity) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }

        double sign = Math.signum(value);
        double magnitude = Math.abs(value);

        magnitude = (magnitude - deadzone) / (1.0 - deadzone);

        double expoValue = expo * magnitude * magnitude + (1.0 - expo) * magnitude;

        double effectiveRate = rcRate + superRate * magnitude * magnitude;

        double output = sign * expoValue * effectiveRate;

        return Math.max(-maxSensitivity, Math.min(maxSensitivity, output));
    }

    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(CAM_Fx, CAM_Fy, CAM_Cx, CAM_Cy)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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

    private DetectedColor getBallRotatorTagColor() {
        //NormalizedRGBA colors = colorSensorBallRotation.getNormalizedColors();

        //double distance = 0;
        //if (colorSensorBallRotation instanceof DistanceSensor) {
        //    distance = ((DistanceSensor)colorSensorBallRotation).getDistance(DistanceUnit.MM);
        //}

        //return detectBallRotatorTagColor(colors, distance);
        return DetectedColor.TAG3; // 임시
    }

    private void ballRotatorTick() {
        DetectedColor detectedColor = getBallRotatorTagColor();
        if (detectedColor != ballRotatorTarget) {
            servoRotateBall.setPower(SERVO_ROTATE_BALL_SPEED);
        } else {
            servoRotateBall.setPower(0);
        }
    }

    private void loadBallTick() {
        if (ballLoadState == BallLoadState.LOADED) {
            long time = ballLoadTimer.time(TimeUnit.MILLISECONDS);
            if (time >= LOAD_BALL_LOADED_WAIT_MS) {
                ballLoadState = BallLoadState.RETURNING;
                servoLoadBall.setPosition(SERVO_LOAD_BALL_IDLE_POSITION);
            }
        } else if ((ballLoadState == BallLoadState.LOADING) && (Math.abs(servoLoadBall.getPosition() - SERVO_LOAD_BALL_ACTIVE_POSITION) <= LOAD_BALL_SERVO_LIMIT)) {
            ballLoadState = BallLoadState.LOADED;
            ballLoadTimer.reset();
        } else if ((ballLoadState == BallLoadState.RETURNING) && (Math.abs(servoLoadBall.getPosition() - SERVO_LOAD_BALL_IDLE_POSITION) <= LOAD_BALL_SERVO_LIMIT)) {
            ballLoadState = BallLoadState.IDLE;
        }
    }

    private void shootBall() {
        ballLoadState = BallLoadState.LOADING;
        servoLoadBall.setPosition(SERVO_LOAD_BALL_ACTIVE_POSITION);
    }

    private void shootTick() {
        motorShoot.setPower(motorShootPower);
    }

    private double calculateBallShootPower() {
        return 0.0;
    }

    private void ballShootAlignmentTick() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        AprilTagDetection bestTag = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    if ((detection.id == AprilTagID.BLUE_GOAL && alliance == Alliance.BLUE)
                            || (detection.id == AprilTagID.READ_GOAL && alliance == Alliance.RED)) {
                        bestTag = detection;
                    }
                }
            }
        }

        if (bestTag != null) {
            double bearing = bestTag.ftcPose.bearing;

            if (Math.abs(bearing) < deadband) {
                servoYaw.setPower(0);
                lastError = 0;
                lastTime = getRuntime();
                return;
            }

            double now = getRuntime();
            double dt = now - lastTime;
            if (dt <= 0) dt = 0.01;

            double derivative = (bearing - lastError) / dt;
            double power = kP * bearing + kD * derivative;

            power = Range.clip(power, minPower, maxPower);

            servoYaw.setPower(-power);

            lastError = bearing;
            lastTime = now;
        } else {
            servoYaw.setPower(0);
        }
    }
}
