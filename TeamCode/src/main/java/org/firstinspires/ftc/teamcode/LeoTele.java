package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drive;

@TeleOp(name = "Leo Tele", group = "Leo")
public class LeoTele extends LinearOpMode {

    private Alliance alliance = Alliance.BLUE;

    private Drive drive;

    private boolean esrTriggered = false;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean manualESR = gamepad1.back && gamepad1.start;
            boolean autoESR = checkAutoESRCondition();

            if (manualESR || autoESR) {
                if (!esrTriggered) {
                    drive.resetSoft();
                    esrTriggered = true;
                }
            } else {
                esrTriggered = false;
            }

            if (!esrTriggered) {
                boolean boost = gamepad1.right_bumper;
                double turnStick = gamepad1.left_stick_x;
                double driveStick = -gamepad1.right_stick_y;

                drive.driveArcade(turnStick, driveStick, boost);
            }

            telemetry.addData("ESR", esrTriggered ? "TRIGGERED" : "normal");
            telemetry.addData("Heading (deg)", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Left ticks", "%.0f", drive.getLeftPositionTicks());
            telemetry.addData("Right ticks", "%.0f", drive.getRightPositionTicks());
            telemetry.update();

            sleep(20);
        }
    }

    private boolean checkAutoESRCondition() {
        return false;
    }
}
