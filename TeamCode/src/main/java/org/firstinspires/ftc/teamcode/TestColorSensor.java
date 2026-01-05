package org.firstinspires.ftc.teamcode;

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

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;

    static final float COLOR_SENSOR_GAIN = 0;

    @Override
    public void runOpMode() {
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor2");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "color_sensor3");

        telemetry.addLine("[Init] hardwareMap initialized");
        telemetry.update();

        colorSensor1.setGain(COLOR_SENSOR_GAIN);
        colorSensor2.setGain(COLOR_SENSOR_GAIN);
        colorSensor3.setGain(COLOR_SENSOR_GAIN);

        telemetry.addLine(String.format(Locale.KOREA, "ColorSensor gain set to %f", COLOR_SENSOR_GAIN));

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

        while (opModeIsActive()) {
            telemetryColor();

            telemetry.addData("Status", "Running");
            telemetry.update();

            sleep(200);
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
