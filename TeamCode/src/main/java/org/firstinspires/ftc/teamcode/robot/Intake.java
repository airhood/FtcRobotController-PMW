package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private static final double PROXIMITY_LOADED_THRESHOLD_MM = 30.0;
    private static final float GREEN_DOMINANT_RATIO = 1.3f;
    private static final double INTAKE_POWER = 0.9;

    private static final double SERVO_IDLE_POSITION = 0.05;
    private static final double SERVO_LIFT_POSITION = 0.3;
    private static final double LOWER_DELAY_SEC = 2.0;

    private static final double REVERSE_POWER = -0.3;
    private static final double REVERSE_POWER_FAST = -0.9;

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final DcMotor intakeMotor;
    private final Servo liftServo;

    private final ElapsedTime notLoadedTimer = new ElapsedTime();
    private boolean servoLifted = false;

    public Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_color");
        distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;

        intakeMotor = hardwareMap.get(DcMotor.class, "motor4");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftServo = hardwareMap.get(Servo.class, "servo2");
        liftServo.setPosition(SERVO_IDLE_POSITION);
    }

    public boolean isArtifactLoaded() {
        if (distanceSensor == null) return false;
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        return distance < PROXIMITY_LOADED_THRESHOLD_MM;
    }

    public Vision.ArtifactColor getLoadedArtifactColor() {
        if (!isArtifactLoaded()) return null;

        NormalizedRGBA rgba = colorSensor.getNormalizedColors();

        if (rgba.green > rgba.red * GREEN_DOMINANT_RATIO) {
            return Vision.ArtifactColor.GREEN;
        }
        return Vision.ArtifactColor.PURPLE;
    }

    public double getProximity() {
        if (distanceSensor == null) return -1;
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public boolean isIdle() {
        return !servoLifted;
    }

    public void update(boolean shooting) {
        if (isArtifactLoaded() && !servoLifted) {
            liftServo.setPosition(SERVO_LIFT_POSITION);
            servoLifted = true;
            if (!shooting) {
                stop();
            }
        }

        if (shooting && servoLifted && !isArtifactLoaded()) {
            liftServo.setPosition(SERVO_IDLE_POSITION);
            servoLifted = false;
        }
    }

    public void feedToShooter() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void reverseSlow() {
        intakeMotor.setPower(REVERSE_POWER);
    }

    public void reverseFast() {
        intakeMotor.setPower(REVERSE_POWER_FAST);
    }

    public void start() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}