package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    private Servo servo;
    private double angle = 0.0;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo2");

        servo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Angle", angle);

            if (gamepad1.y) {
                angle += 0.002;
            } else if (gamepad1.a) {
                angle -= 0.002;
            }

            servo.setPosition(angle);

            telemetry.update();

            sleep(20);
        }
    }
}
