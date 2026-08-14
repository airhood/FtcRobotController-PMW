package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@Autonomous(name = "Leo Auto")
public class LeoAuto extends LinearOpMode {

    private enum State {
        SEARCH_ARTIFACT,
        APPROACH_ARTIFACT,
        AIM_AT_GOAL,
        SHOOT,
        PARK,
        DONE
    }

    private Alliance alliance = Alliance.BLUE;

    private Vision.Motif targetMotif = null;
    private State state = State.SEARCH_ARTIFACT;

    private int consecutiveBlobMissCount = 0;
    private static final int BLOB_MISS_TOLERANCE_TICKS = 8;
    private double lastKnownBlobX = 0.0;

    private Drive drive;
    private Vision vision;
    private Intake intake;
    private Shooter shooter;
    private Localizer localizer;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private static final String TAG_WEBCAM_NAME = "Webcam 1";
    private static final String ARTIFACT_WEBCAM_NAME = "Webcam 2";
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private static final double START_X_MM = 0;
    private static final double START_Y_MM = 0;
    private static final double START_HEADING_DEG = 0;

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
    private static final double PARK_DRIVE_POWER = 0.4;
    private static final double PARK_DURATION_SEC = 1.0;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, TAG_WEBCAM_NAME, ARTIFACT_WEBCAM_NAME, CAMERA_POSITION, CAMERA_ORIENTATION);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        localizer = new Localizer(START_X_MM, START_Y_MM, START_HEADING_DEG);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !gamepad1.b) alliance = Alliance.BLUE;
            else if (!gamepad1.x && gamepad1.b) alliance = Alliance.RED;

            updateTargetMotifIfDetected();

            telemetry.addData("Alliance (X=Blue, B=Red)", alliance);
            telemetry.addData("Target Motif", targetMotif != null ? targetMotif : "(not detected)");
            telemetry.update();
        }

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            localizer.predict(drive);
            updateLocalizerFromVision();
            updateTargetMotifIfDetected();

            runStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Target Motif", targetMotif != null ? targetMotif : "(not detected)");
            telemetry.addData("Heading", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Localizer (x,y,theta)", "%.0f, %.0f, %.1f",
                    localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
            telemetry.addData("Localizer uncertainty (x,y)", "%.1f, %.1f",
                    localizer.getUncertaintyX(), localizer.getUncertaintyY());
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        vision.close();
    }

    private void runStateMachine() {
        switch (state) {
            case SEARCH_ARTIFACT:
                handleSearchArtifact();
                break;
            case APPROACH_ARTIFACT:
                handleApproachArtifact();
                break;
            case AIM_AT_GOAL:
                handleAimAtGoal();
                break;
            case SHOOT:
                handleShoot();
                break;
            case PARK:
                handlePark();
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

    private void updateLocalizerFromVision() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);
        if (goalTag == null) return;

        double tagFieldX = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_X_MM : GOAL_RED_FIELD_X_MM;
        double tagFieldY = (alliance == Alliance.BLUE) ? GOAL_BLUE_FIELD_Y_MM : GOAL_RED_FIELD_Y_MM;

        localizer.update(goalTag, tagFieldX, tagFieldY);
    }

    private void updateTargetMotifIfDetected() {
        Vision.Motif detectedMotif = vision.getTargetMotif();
        if (detectedMotif != null) {
            targetMotif = detectedMotif;
        }
    }

    private void handleSearchArtifact() {
        ColorBlobLocatorProcessor.Blob blob = vision.getLargestArtifactBlobAnyColor();

        if (blob != null) {
            drive.stop();
            transitionTo(State.APPROACH_ARTIFACT);
            return;
        }

        // TODO: 회전 방향/범위가 미정. 지금은 한 방향으로만 계속 회전.
        drive.setPowerRaw(-SEARCH_TURN_POWER, SEARCH_TURN_POWER);
    }

    private void handleApproachArtifact() {
        intake.start();

        if (intake.isArtifactLoaded()) {
            intake.stop();
            drive.stop();
            transitionTo(State.AIM_AT_GOAL);
            return;
        }

        ColorBlobLocatorProcessor.Blob blob = vision.getLargestArtifactBlobAnyColor();
        double blobX;

        if (blob == null) {
            consecutiveBlobMissCount++;
            if (consecutiveBlobMissCount > BLOB_MISS_TOLERANCE_TICKS) {
                consecutiveBlobMissCount = 0;
                transitionTo(State.SEARCH_ARTIFACT);
                return;
            }
            blobX = lastKnownBlobX;
        } else {
            consecutiveBlobMissCount = 0;
            blobX = vision.getBlobNormalizedX(blob);
            lastKnownBlobX = blobX;
        }

        // TODO: 미검증 임시 로직. 실제 로봇에서 게인/파워값 재조정 필요.
        double turnCorrection = blobX * 0.3;
        double leftPower = APPROACH_DRIVE_POWER + turnCorrection;
        double rightPower = APPROACH_DRIVE_POWER - turnCorrection;

        drive.setPowerRaw(leftPower, rightPower);
    }

    private void handleAimAtGoal() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);

        if (goalTag == null || goalTag.ftcPose == null) {
            // TODO: 탐색 범위/방향 미정
            drive.setPowerRaw(-SEARCH_TURN_POWER * 0.5, SEARCH_TURN_POWER * 0.5);
            return;
        }

        double bearingError = goalTag.ftcPose.bearing;
        double range = goalTag.ftcPose.range;

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

        double range = vision.getRangeToTag(
                alliance == Alliance.RED ? Vision.TAG_ID_GOAL_RED : Vision.TAG_ID_GOAL_BLUE);
        shooter.setPowerForDistance(range);

        if (stateTimer.seconds() > SHOOT_DURATION_SEC) {
            shooter.stop();
            transitionTo(State.PARK);
        }
    }

    private void handlePark() {
        if (stateTimer.seconds() > PARK_DURATION_SEC) {
            drive.stop();
            transitionTo(State.DONE);
            return;
        }

        drive.setPowerRaw(PARK_DRIVE_POWER, PARK_DRIVE_POWER);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}