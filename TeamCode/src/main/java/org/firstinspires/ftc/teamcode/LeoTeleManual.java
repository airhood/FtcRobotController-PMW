package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Intake;

@TeleOp(name = "Leo Tele Manual Intake", group = "Leo")
public class LeoTeleManual extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.2;
    private static final double SERVO_IDLE_POSITION = 0.05;
    private static final double SERVO_LIFT_POSITION = 0.3;

    private Alliance alliance = Alliance.BLUE;
    private Drive drive;
    private Intake intake;
    private Servo liftServo;
    private boolean esrTriggered = false;

    private boolean servoToggledUp = false;
    private boolean servoButtonLast = false;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        liftServo = hardwareMap.get(Servo.class, "servo2");

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
                double turnStick = gamepad1.left_stick_x;
                double driveStick = -gamepad1.right_stick_y;
                drive.driveArcade(turnStick, driveStick, false);
            }

            if (gamepad1.y) {
                intake.feedToShooter();
            } else if (gamepad1.x) {
                intake.reverseFast();
            } else if (gamepad1.b) {
                intake.reverseSlow();
            } else {
                boolean intakeHeld = gamepad1.right_bumper || (gamepad1.right_trigger > TRIGGER_THRESHOLD);
                if (intakeHeld) {
                    intake.start();
                } else {
                    intake.stop();
                }
            }

            boolean servoButton = gamepad1.left_bumper || (gamepad1.left_trigger > TRIGGER_THRESHOLD);
            if (servoButton && !servoButtonLast) {
                servoToggledUp = !servoToggledUp;
                liftServo.setPosition(servoToggledUp ? SERVO_LIFT_POSITION : SERVO_IDLE_POSITION);
            }
            servoButtonLast = servoButton;

            telemetry.addData("ESR", esrTriggered ? "TRIGGERED" : "normal");
            telemetry.addData("Heading (deg)", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Intake proximity", "%.1f", intake.getProximity());
            telemetry.addData("Servo toggled", servoToggledUp);
            telemetry.update();

            sleep(20);
        }
    }

    private boolean checkAutoESRCondition() {
        return false;
    }
}