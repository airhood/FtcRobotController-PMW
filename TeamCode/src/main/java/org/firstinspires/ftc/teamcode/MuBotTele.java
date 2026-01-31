package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MuBot Tele", group = "MuBot")
public class MuBotTele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo servo1;

    private boolean enablePositionHolding = false;
    private double targetHeading = 0.0;
    private double targetPositionY = 0.0;

    // Heading PID constants
    static final double KP_HEADING = 0.02;
    static final double KI_HEADING = 0.0001;
    static final double KD_HEADING = 0.001;

    // Position PID constants
    static final double KP_POSITION = 0.0008;
    static final double KI_POSITION = 0.00005;
    static final double KD_POSITION = 0.0002;

    private double headingIntegralSum = 0.0;
    private double headingLastError = 0.0;

    private double positionIntegralSum = 0.0;
    private double positionLastError = 0.0;

    private ElapsedTime pidTimer = new ElapsedTime();

    private static final double DRIVE_SPEED_DEADZONE = 0.10;
    private static final double DRIVE_SPEED_EXPO = 0.3;
    private static final double DRIVE_SPEED_RC_RATE = 0.7;
    private static final double DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    private static final double TURN_SPEED_DEADZONE = 0.05;
    private static final double TURN_SPEED_EXPO = 0.5;
    private static final double TURN_SPEED_RC_RATE = 0.20;
    private static final double TURN_SPEED_MAX_SENSITIVITY = 0.5;

    private boolean boost = false;
    private static final double BOOST_DRIVE_SPEED_DEADZONE = 0.10;
    private static final double BOOST_DRIVE_SPEED_EXPO = 0.3;
    private static final double BOOST_DRIVE_SPEED_RC_RATE = 1.0;
    private static final double BOOST_DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorRight = hardwareMap.get(DcMotor.class, "motor2");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParameters);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1.setPosition(0.05);

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            if (enablePositionHolding) {
                positionHolding();
            } else {
                moveWheel();
            }
            controlServo();

            boost = gamepad1.right_bumper;

            telemetry.addData("Boost", "%b", boost);

            telemetry.addData("Servo Pos", "%.2f", servo1.getPosition());
            telemetry.addData("Position Holding", enablePositionHolding);
            telemetry.addData("Current Heading", "%.2f°", getCurrentHeading());
            telemetry.addData("Target Heading", "%.2f°", targetHeading);
            telemetry.addData("Current Position", "%.0f ticks", getCurrentPosition());
            telemetry.addData("Target Position", "%.0f ticks", targetPositionY);
            telemetry.addData("Position Error", "%.0f ticks", targetPositionY - getCurrentPosition());
            telemetry.update();

            sleep(20);
        }
    }

    private void moveWheel() {
        double leftX = processStickInput(gamepad1.left_stick_x, TURN_SPEED_DEADZONE, TURN_SPEED_EXPO, TURN_SPEED_RC_RATE, 0.2, TURN_SPEED_MAX_SENSITIVITY);
        double rightY = 0;
        if (boost) {
            rightY = processStickInput(gamepad1.right_stick_y, BOOST_DRIVE_SPEED_DEADZONE, BOOST_DRIVE_SPEED_EXPO, BOOST_DRIVE_SPEED_RC_RATE, 0.0, BOOST_DRIVE_SPEED_MAX_SENSITIVITY);
            rightY *= -1;
        } else {
            rightY = processStickInput(gamepad1.right_stick_y, DRIVE_SPEED_DEADZONE, DRIVE_SPEED_EXPO, DRIVE_SPEED_RC_RATE, 0.0, DRIVE_SPEED_MAX_SENSITIVITY);
            rightY *= -1;
        }

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

    private void controlServo() {
        if (gamepad1.leftBumperWasPressed()) {
            if (!enablePositionHolding) {
                servo1.setPosition(0.8);
                targetHeading = getCurrentHeading();
                targetPositionY = getCurrentPosition(); // 현재 엔코더 위치를 목표로 설정

                // PID 초기화
                headingIntegralSum = 0.0;
                headingLastError = 0.0;
                positionIntegralSum = 0.0;
                positionLastError = 0.0;

                pidTimer.reset();
                enablePositionHolding = true;
            } else {
                enablePositionHolding = false;
            }
        }
    }

    public static double processStickInput(double value, double deadzone, double expo,
                                           double rcRate, double superRate, double maxSensitivity) {
        if (Math.abs(value) < deadzone) return 0.0;
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        magnitude = (magnitude - deadzone) / (1.0 - deadzone);
        double expoValue = expo * magnitude * magnitude + (1.0 - expo) * magnitude;
        double effectiveRate = rcRate + superRate * magnitude * magnitude;
        double output = sign * expoValue * effectiveRate;
        return Math.max(-maxSensitivity, Math.min(maxSensitivity, output));
    }

    private double getCurrentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double getCurrentPosition() {
        return (motorLeft.getCurrentPosition() + motorRight.getCurrentPosition()) / 2.0;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void positionHolding() {
        double deltaTime = pidTimer.seconds();
        if (deltaTime < 0.001) return;
        pidTimer.reset();

        // Heading PID (IMU 기반 - 정확함)
        double currentHeading = getCurrentHeading();
        double headingError = normalizeAngle(targetHeading - currentHeading);

        headingIntegralSum += headingError * deltaTime;
        headingIntegralSum = Math.max(-10.0, Math.min(10.0, headingIntegralSum));

        double headingDerivative = (headingError - headingLastError) / deltaTime;
        double headingCorrection = KP_HEADING * headingError
                + KI_HEADING * headingIntegralSum
                + KD_HEADING * headingDerivative;
        headingLastError = headingError;

        // Position PID (엔코더 기반 - 슬립 있지만 어느정도 유용)
        double currentPosition = getCurrentPosition();
        double positionError = targetPositionY - currentPosition;

        positionIntegralSum += positionError * deltaTime;
        positionIntegralSum = Math.max(-2000.0, Math.min(2000.0, positionIntegralSum));

        double positionDerivative = (positionError - positionLastError) / deltaTime;
        double positionCorrection = KP_POSITION * positionError
                + KI_POSITION * positionIntegralSum
                + KD_POSITION * positionDerivative;
        positionLastError = positionError;

        // 보정값 제한
        headingCorrection = Math.max(-0.5, Math.min(0.5, headingCorrection));
        positionCorrection = Math.max(-0.6, Math.min(0.6, positionCorrection));

        // 전후방 + 회전 보정 결합
        double leftPower = positionCorrection - headingCorrection;
        double rightPower = positionCorrection + headingCorrection;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }
}