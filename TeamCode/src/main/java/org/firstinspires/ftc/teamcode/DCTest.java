package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DC Test")
public class DCTest extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.right_stick_y);

            sleep(20);
        }
    }
}
