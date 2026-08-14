package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Leo Auto Aim Test", group = "Test")
public class LeoAutoAimTest extends LinearOpMode {

    private enum State {
        RANDOM_MOVE,
        AIM_AT_GOAL,
        SHOOT,
        DONE
    }

    private Alliance alliance = Alliance.BLUE;
    private State state = State.RANDOM_MOVE;

    private Drive drive;
    private Vision vision;
    private Localizer localizer;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private double lastSeenRangeRaw = -1;
    private double lastSeenBearing = 0;

    private static final String TAG_WEBCAM_NAME = "Webcam 1";
    private static final String ARTIFACT_WEBCAM_NAME = "Webcam 2";
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private static final double START_X_MM = 0;
    private static final double START_Y_MM = 0;
    private static final double START_HEADING_DEG = 90;

    private static final double GOAL_BLUE_FIELD_X_MM = -1524;
    private static final double GOAL_BLUE_FIELD_Y_MM = 1524;
    private static final double GOAL_RED_FIELD_X_MM = 1524;
    private static final double GOAL_RED_FIELD_Y_MM = 1524;

    private static final double SEARCH_TURN_POWER = 0.25;
    private static final double APPROACH_DRIVE_POWER = 0.35;
    private static final double GOAL_HEADING_TOLERANCE_DEG = 3.0;
    private static final double SHOOT_SPOT_RANGE_MIN_MM = 800;
    private static final double SHOOT_SPOT_RANGE_MAX_MM = 1200;
    private static final double SHOOT_DURATION_SEC = 1.5;
    
    private static final double RANDOM_MOVE_BACKWARD_SEC = 1.5;
    private static final double RANDOM_MOVE_TURN_SEC = 1.0;
    private static final double RANDOM_MOVE_FORWARD_SEC = 1.2;
    private static final double RANDOM_MOVE_POWER = 0.35;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, TAG_WEBCAM_NAME, ARTIFACT_WEBCAM_NAME, CAMERA_POSITION, CAMERA_ORIENTATION);
        localizer = new Localizer(START_X_MM, START_Y_MM, START_HEADING_DEG);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !gamepad1.b) alliance = Alliance.BLUE;
            else if (!gamepad1.x && gamepad1.b) alliance = Alliance.RED;

            telemetry.addData("Alliance (X=Blue, B=Red)", alliance);
            telemetry.update();
        }

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            localizer.predict(drive);
            updateLocalizerFromVision();

            runStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Heading", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Localizer (x,y,theta)", "%.0f, %.0f, %.1f",
                    localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
            telemetry.addData("Localizer uncertainty (x,y)", "%.1f, %.1f",
                    localizer.getUncertaintyX(), localizer.getUncertaintyY());
            telemetry.addData("Last seen range (raw, no conversion)", "%.2f", lastSeenRangeRaw);
            telemetry.addData("Last seen bearing (raw)", "%.2f", lastSeenBearing);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        vision.close();
    }

    private void runStateMachine() {
        switch (state) {
            case RANDOM_MOVE:
                handleRandomMove();
                break;
            case AIM_AT_GOAL:
                handleAimAtGoal();
                break;
            case SHOOT:
                handleShoot();
                break;
            case DONE:
                drive.stop();
                break;
        }
    }

    private void transitionTo(State next) {
        state = next;
        stateTimer.reset();
    }

    private void handleRandomMove() {
        double t = stateTimer.seconds();

        if (t < RANDOM_MOVE_BACKWARD_SEC) {
            drive.setPowerRaw(-RANDOM_MOVE_POWER, -RANDOM_MOVE_POWER);
        } else if (t < RANDOM_MOVE_BACKWARD_SEC + RANDOM_MOVE_TURN_SEC) {
            drive.setPowerRaw(RANDOM_MOVE_POWER, -RANDOM_MOVE_POWER);
        } else if (t < RANDOM_MOVE_BACKWARD_SEC + RANDOM_MOVE_TURN_SEC + RANDOM_MOVE_FORWARD_SEC) {
            drive.setPowerRaw(RANDOM_MOVE_POWER, RANDOM_MOVE_POWER);
        } else {
            drive.stop();
            transitionTo(State.AIM_AT_GOAL);
        }
    }

    private void updateLocalizerFromVision() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);
        if (goalTag == null) return;

        double tagFieldX = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_X_MM : GOAL_RED_FIELD_X_MM;
        double tagFieldY = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_Y_MM : GOAL_RED_FIELD_Y_MM;

        localizer.update(goalTag, tagFieldX, tagFieldY);
    }

    private void handleAimAtGoal() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);

        if (goalTag == null || goalTag.ftcPose == null) {
            double goalFieldX = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_X_MM : GOAL_RED_FIELD_X_MM;
            double goalFieldY = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_Y_MM : GOAL_RED_FIELD_Y_MM;

            double dx = goalFieldX - localizer.getX();
            double dy = goalFieldY - localizer.getY();
            double desiredHeading = Math.toDegrees(Math.atan2(dy, dx));
            double headingError = normalizeAngle(desiredHeading - localizer.getHeadingDeg());

            telemetry.addData("tag", "not visible, searching toward EKF estimate");
            telemetry.addData("desiredHeading", "%.1f", desiredHeading);
            telemetry.addData("headingErrorToGoal", "%.1f", headingError);

            if (Math.abs(headingError) < GOAL_HEADING_TOLERANCE_DEG) {
                drive.stop();
            } else {
                double turnDirection = (headingError >= 0) ? 1.0 : -1.0;
                drive.setPowerRaw(-turnDirection * SEARCH_TURN_POWER * 0.5, turnDirection * SEARCH_TURN_POWER * 0.5);
            }
            return;
        }

        double bearingErrorRaw = goalTag.ftcPose.bearing;
        double rangeRaw = goalTag.ftcPose.range;

        lastSeenRangeRaw = rangeRaw;
        lastSeenBearing = bearingErrorRaw;

        double bearingError = bearingErrorRaw;
        double range = rangeRaw * 25.4;

        telemetry.addData("bearing", "%.1f", bearingError);
        telemetry.addData("range (converted mm)", "%.1f", range);
        telemetry.addData("range (raw)", "%.2f", rangeRaw);

        boolean inShootSpotRange = range >= SHOOT_SPOT_RANGE_MIN_MM && range <= SHOOT_SPOT_RANGE_MAX_MM;
        boolean headingAligned = Math.abs(bearingError) < GOAL_HEADING_TOLERANCE_DEG;

        if (inShootSpotRange && headingAligned) {
            drive.stop();
            transitionTo(State.SHOOT);
            return;
        }

        if (!inShootSpotRange) {
            double driveDirection = (range > SHOOT_SPOT_RANGE_MAX_MM) ? 1.0 : -1.0;
            double turnCorrection = clamp(bearingError * 0.02, -0.2, 0.2);
            drive.setPowerRaw(driveDirection * APPROACH_DRIVE_POWER - turnCorrection, driveDirection * APPROACH_DRIVE_POWER + turnCorrection);
            return;
        }

        double turnPower = clamp(bearingError * 0.02, -0.3, 0.3);
        drive.setPowerRaw(-turnPower, turnPower);
    }

    private void handleShoot() {
        drive.stop();

        if (stateTimer.seconds() > SHOOT_DURATION_SEC) {
            transitionTo(State.DONE);
        }
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}