package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Servo Range Test", group = "Test")
public class ServoRangeTest extends LinearOpMode {

    private Servo servo3;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private double servoPosition = -1;
    private static final double SERVO_STEP_PER_TICK = 0.002;

    @Override
    public void runOpMode() {
        servo3 = hardwareMap.get(Servo.class, "servo2");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_color");
        distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;

        telemetry.addData("Status", "Initialized - servo NOT commanded yet");
        telemetry.addLine("A: decrease slowly (hold), B: increase slowly (hold)");
        telemetry.addLine("Release button immediately if anything binds or resists");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                if (servoPosition < 0) servoPosition = 0.5;
                servoPosition = Math.min(1.0, servoPosition + SERVO_STEP_PER_TICK);
                servo3.setPosition(servoPosition);
            } else if (gamepad1.a) {
                if (servoPosition < 0) servoPosition = 0.5;
                servoPosition = Math.max(0.0, servoPosition - SERVO_STEP_PER_TICK);
                servo3.setPosition(servoPosition);
            }

            double distanceMM = (distanceSensor != null) ? distanceSensor.getDistance(DistanceUnit.MM) : -1;
            NormalizedRGBA rgba = colorSensor.getNormalizedColors();

            telemetry.addData("Servo commanded position", servoPosition < 0 ? "not yet moved" : String.format("%.3f", servoPosition));
            telemetry.addData("Distance (mm)", "%.1f", distanceMM);
            telemetry.addData("Color R/G/B", "%.3f / %.3f / %.3f", rgba.red, rgba.green, rgba.blue);
            telemetry.addLine();
            telemetry.addLine("Hold A: move down slowly | Hold B: move up slowly");
            telemetry.update();

            sleep(20);
        }
    }
}