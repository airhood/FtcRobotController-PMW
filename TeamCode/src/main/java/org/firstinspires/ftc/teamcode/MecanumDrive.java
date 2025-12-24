package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Mecanum Drive")
public class MecanumDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private Servo servo1;
    private Servo servo2;

    private Position cameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_DIAMETER_MM = 40;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);

    static final double DRIVE_SPEED_DEADZONE = 0.05;
    static final double DRIVE_SPEED_EXPO = 0.3;
    static final double DRIVE_SPEED_RC_RATE = 1.0;
    static final double DRIVE_SPEED_SUPER_RATE = 0.0;
    static final double DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    static final double TURN_SPEED_DEADZONE = 0.05;
    static final double TURN_SPEED_EXPO = 0.5;
    static final double TURN_SPEED_RC_RATE = 0.8;
    static final double TURN_SPEED_SUPER_RATE = 0.3;
    static final double TURN_SPEED_MAX_SENSITIVITY = 0.7;

    static final boolean WEB_CAM_MANUAL_EXPOSURE = true;
    static final int WEB_CAM_EXPOSURE_MS = 6;
    static final int WEB_CAM_GAIN = 250;

    static final float COLOR_SENSOR_GAIN = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor2");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor3");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor4");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");

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

        colorSensor1.setGain(COLOR_SENSOR_GAIN);
        colorSensor2.setGain(COLOR_SENSOR_GAIN);
        colorSensor3.setGain(COLOR_SENSOR_GAIN);

        telemetry.addLine(String.format("ColorSensor gain set to %f", COLOR_SENSOR_GAIN));

        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor1).enableLight(true);
            telemetry.addLine("ColorSensor1 light enabled");
        }
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor2).enableLight(true);
            telemetry.addLine("ColorSensor2 light enabled");
        }
        if (colorSensor3 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor3).enableLight(true);
            telemetry.addLine("ColorSensor3 light enabled");
        }
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        double servoPos = 0.0;
        while (opModeIsActive()) {
            telemetryAprilTag();

            moveWheel();

            if (gamepad1.b) {
                servoPos += 0.004;
            } else if (gamepad1.x) {
                servoPos -= 0.004;
            }
            if (servoPos >= 1) {
                servoPos -= 1;
            }

            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);

            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }

    private void moveWheel() {
        double leftX = processStickInput(gamepad1.left_stick_x, TURN_SPEED_DEADZONE, TURN_SPEED_EXPO, TURN_SPEED_RC_RATE, TURN_SPEED_SUPER_RATE, TURN_SPEED_MAX_SENSITIVITY);
        double rightX = processStickInput(gamepad1.right_stick_x, DRIVE_SPEED_DEADZONE, DRIVE_SPEED_EXPO, DRIVE_SPEED_RC_RATE, DRIVE_SPEED_SUPER_RATE, DRIVE_SPEED_MAX_SENSITIVITY);
        double rightY = processStickInput(gamepad1.right_stick_y, DRIVE_SPEED_DEADZONE, DRIVE_SPEED_EXPO, DRIVE_SPEED_RC_RATE, DRIVE_SPEED_SUPER_RATE, DRIVE_SPEED_MAX_SENSITIVITY);

        double frontLeft = rightY + rightX + leftX;
        double frontRight = rightY - rightX - leftX;
        double backLeft = rightY - rightX + leftX;
        double backRight = rightY + rightX - leftX;

        double maxMagnitude = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight))
        );

        if (maxMagnitude > 1.0) {
            frontLeft /= maxMagnitude;
            frontRight /= maxMagnitude;
            backLeft /= maxMagnitude;
            backRight /= maxMagnitude;
        }

        motorFrontLeft.setPower(frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(backLeft);
        motorBackRight.setPower(backRight);
    }

    private void encoderDrive(MotorAction frontLeft, MotorAction frontRight,
                              MotorAction backLeft, MotorAction backRight,
                              double timeout) {
        if (opModeIsActive()) {
            int newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(frontLeft.target * COUNTS_PER_MM);
            int newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int)(frontRight.target * COUNTS_PER_MM);
            int newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int)(backLeft.target * COUNTS_PER_MM);
            int newBackRightTarget = motorBackRight.getCurrentPosition() + (int)(backRight.target * COUNTS_PER_MM);

            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorBackLeft.setTargetPosition(newBackLeftTarget);
            motorBackRight.setTargetPosition(newBackRightTarget);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            motorFrontLeft.setPower(frontLeft.speed);
            motorFrontRight.setPower(frontRight.speed);
            motorBackLeft.setPower(backLeft.speed);
            motorBackRight.setPower(backRight.speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() &&
                            motorBackLeft.isBusy() && motorBackRight.isBusy())) {
                telemetry.addData("Running to", " %7d | %7d | %7d | %7d",
                        newFrontLeftTarget, newFrontRightTarget,
                        newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " %7d | %7d | %7d | %7d",
                        motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(),
                        motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
                telemetry.update();
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (mm)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f  (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    private void detectTargetAprilTag(int desired_tag_id) {
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();

        float[] hsvValues1 = new float[3];
        float[] hsvValues2 = new float[3];
        float[] hsvValues3 = new float[3];

        double distance1 = 0;
        double distance2 = 0;
        double distance3 = 0;

        if (colorSensor1 instanceof DistanceSensor) {
            distance1 = ((DistanceSensor)colorSensor1).getDistance(DistanceUnit.MM);
        }
        if (colorSensor2 instanceof DistanceSensor) {
            distance2 = ((DistanceSensor)colorSensor2).getDistance(DistanceUnit.MM);
        }
        if (colorSensor3 instanceof DistanceSensor) {
            distance3 = ((DistanceSensor)colorSensor3).getDistance(DistanceUnit.MM);
        }

        telemetry.addLine("Color1")
                .addData("Red", "%.3f", colors1.red)
                .addData("Green", "%.3f", colors1.green)
                .addData("Blue", "%.3f", colors1.blue)
                .addData("Hue", "%.3f", hsvValues1[0])
                .addData("Saturation", "%.3f", hsvValues1[1])
                .addData("Value" ,"%.3f", hsvValues1[2])
                .addData("Alpha", "%.3f", colors1.alpha)
                .addData("Distance (mm)", "%.3f", distance1);

        telemetry.addLine("Color2")
                .addData("Red", "%.3f", colors2.red)
                .addData("Green", "%.3f", colors2.green)
                .addData("Blue", "%.3f", colors2.blue)
                .addData("Hue", "%.3f", hsvValues2[0])
                .addData("Saturation", "%.3f", hsvValues2[1])
                .addData("Value" ,"%.3f", hsvValues2[2])
                .addData("Alpha", "%.3f", colors2.alpha)
                .addData("Distance (mm)", "%.3f", distance2);

        telemetry.addLine("Color3")
                .addData("Red", "%.3f", colors3.red)
                .addData("Green", "%.3f", colors3.green)
                .addData("Blue", "%.3f", colors3.blue)
                .addData("Hue", "%.3f", hsvValues3[0])
                .addData("Saturation", "%.3f", hsvValues3[1])
                .addData("Value" ,"%.3f", hsvValues3[2])
                .addData("Alpha", "%.3f", colors3.alpha)
                .addData("Distance (mm)", "%.3f", distance3);
    }
}