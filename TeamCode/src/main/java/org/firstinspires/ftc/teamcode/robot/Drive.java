package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {

    private final IMU imu;

    private final DcMotor motorLeft;
    private final DcMotor motorRight;

    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    private static final double DRIVE_SPEED_DEADZONE = 0.10;
    private static final double DRIVE_SPEED_EXPO = 0.3;
    private static final double DRIVE_SPEED_RC_RATE = 0.7;
    private static final double DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    private static final double TURN_SPEED_DEADZONE = 0.05;
    private static final double TURN_SPEED_EXPO = 0.5;
    private static final double TURN_SPEED_RC_RATE = 0.20;
    private static final double TURN_SPEED_MAX_SENSITIVITY = 0.5;

    private static final double BOOST_DRIVE_SPEED_DEADZONE = 0.10;
    private static final double BOOST_DRIVE_SPEED_EXPO = 0.3;
    private static final double BOOST_DRIVE_SPEED_RC_RATE = 1.0;
    private static final double BOOST_DRIVE_SPEED_MAX_SENSITIVITY = 1.0;

    public Drive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
        motorRight = hardwareMap.get(DcMotor.class, "motor2");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                HUB_LOGO_DIRECTION,
                HUB_USB_DIRECTION
        ));
        imu.initialize(imuParameters);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveArcade(double turnStick, double driveStick, boolean boost) {
        double turn = processStickInput(turnStick, TURN_SPEED_DEADZONE, TURN_SPEED_EXPO,
                TURN_SPEED_RC_RATE, 0.2, TURN_SPEED_MAX_SENSITIVITY);

        double forward;
        if (boost) {
            forward = processStickInput(driveStick, BOOST_DRIVE_SPEED_DEADZONE, BOOST_DRIVE_SPEED_EXPO,
                    BOOST_DRIVE_SPEED_RC_RATE, 0.0, BOOST_DRIVE_SPEED_MAX_SENSITIVITY);
        } else {
            forward = processStickInput(driveStick, DRIVE_SPEED_DEADZONE, DRIVE_SPEED_EXPO,
                    DRIVE_SPEED_RC_RATE, 0.0, DRIVE_SPEED_MAX_SENSITIVITY);
        }

        setPowerRaw(forward + turn, forward - turn);
    }

    public void setPowerRaw(double leftPower, double rightPower) {
        double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxMagnitude > 1.0) {
            leftPower /= maxMagnitude;
            rightPower /= maxMagnitude;
        }
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void resetSoft() {
        stop();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                HUB_LOGO_DIRECTION,
                HUB_USB_DIRECTION
        ));
        imu.initialize(imuParameters);
    }

    public double getCurrentHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getLeftPositionTicks() {
        return motorLeft.getCurrentPosition();
    }

    public double getRightPositionTicks() {
        return motorRight.getCurrentPosition();
    }

    public double getAveragePositionTicks() {
        return (getLeftPositionTicks() + getRightPositionTicks()) / 2.0;
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
        return clamp(output, -maxSensitivity, maxSensitivity);
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
