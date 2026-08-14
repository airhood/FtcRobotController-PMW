package org.firstinspires.ftc.teamcode;

public class InputProcessor {
    static double drive_speed_deadzone = 0.05;
    static double drive_speed_expo = 0.3;
    static double drive_speed_rc_rate = 0.7;
    static double drive_speed_super_rate = 0.0;
    static double drive_speed_max_sensitivity = 1.0;

    static double turn_speed_deadzone = 0.05;
    static double turn_speed_expo = 0.5;
    static double turn_speed_rc_rate = 0.25;
    static double turn_speed_super_rate = 0.3;
    static double turn_speed_max_sensitivity = 0.7;

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
}
