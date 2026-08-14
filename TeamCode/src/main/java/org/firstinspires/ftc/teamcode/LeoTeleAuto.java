package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Intake;

@TeleOp(name = "Leo Tele Auto Intake", group = "Leo")
public class LeoTeleAuto extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.2;

    private Alliance alliance = Alliance.BLUE;
    private Drive drive;
    private Intake intake;
    private boolean esrTriggered = false;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);

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

            intake.update(false);

            boolean intakeHeld = gamepad1.right_bumper || (gamepad1.right_trigger > TRIGGER_THRESHOLD);
            if (intakeHeld) {
                intake.start();
            } else {
                intake.stop();
            }

            telemetry.addData("ESR", esrTriggered ? "TRIGGERED" : "normal");
            telemetry.addData("Heading (deg)", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Intake idle", intake.isIdle());
            telemetry.addData("Intake proximity", "%.1f", intake.getProximity());
            telemetry.update();

            sleep(20);
        }
    }

    private boolean checkAutoESRCondition() {
        return false;
    }
}