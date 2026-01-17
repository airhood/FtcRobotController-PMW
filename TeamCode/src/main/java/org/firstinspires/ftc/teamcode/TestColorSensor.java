package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "Test Color Sensor")
public class TestColorSensor extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    static final float COLOR_SENSOR_GAIN = 0;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        telemetry.addLine("[Init] hardwareMap initialized");
        telemetry.update();

        colorSensor.setGain(COLOR_SENSOR_GAIN);

        telemetry.addLine(String.format(Locale.KOREA, "ColorSensor gain set to %f", COLOR_SENSOR_GAIN));

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
            telemetry.addLine("ColorSensor light enabled");
        }
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetryColor();

            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(200);
        }
    }

    private void telemetryColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        double distance = 0;

        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
        }

        telemetry.addLine("Color1")
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colors.alpha)
                .addData("Distance (mm)", "%.3f", distance);
    }
}
