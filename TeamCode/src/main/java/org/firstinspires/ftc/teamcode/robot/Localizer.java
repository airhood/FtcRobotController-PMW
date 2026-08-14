package org.firstinspires.ftc.teamcode.robot;

import android.mtp.MtpConstants;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

public class Localizer {

    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEAR_REDUCTION = 19.2;
    private static final double WHEEL_DIAMETER_MM = 95;
    private static final double MM_PER_TICK = (WHEEL_DIAMETER_MM * Math.PI) / (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

    private static final double PROCESS_NOISE_XY = 4.0;
    private static final double PROCESS_NOISE_THETA = 0.5;

    private static final double BASE_OBSERVATION_NOISE_XY = 30.0;
    private static final double BASE_OBSERVATION_NOISE_THETA = 5.0;
    private static final double OBSERVATION_NOISE_REFERENCE_RANGE_MM = 1000.0;

    private static final double OUTLIER_GATE_SIGMA = 3.0;

    private double x;
    private double y;
    private double theta;

    private double pXX, pYY, pThetaTheta;

    private double lastAverageTicks;
    private boolean initialized = false;

    public Localizer(double startX, double startY, double startTheta) {
        this.x = startX;
        this.y = startY;
        this.theta = startTheta;

        this.pXX = 10.0;
        this.pYY = 10.0;
        this.pThetaTheta = 5.0;
    }

    public void predict(Drive drive) {
        double currentAverageTicks = (drive.getLeftPositionTicks() + drive.getRightPositionTicks()) / 2.0;
        double currentHeading = drive.getCurrentHeadingDeg();

        if (!initialized) {
            lastAverageTicks = currentAverageTicks;
            theta = currentHeading;
            initialized = true;
            return;
        }

        double deltaTicks = currentAverageTicks - lastAverageTicks;
        lastAverageTicks = currentAverageTicks;

        double deltaDistanceMM = deltaTicks * MM_PER_TICK;

        theta = currentHeading;

        double headingRad = Math.toRadians(theta);
        x += deltaDistanceMM * Math.cos(headingRad);
        y += deltaDistanceMM * Math.sin(headingRad);

        pXX += PROCESS_NOISE_XY * Math.abs(deltaDistanceMM);
        pYY += PROCESS_NOISE_XY * Math.abs(deltaDistanceMM);
        pThetaTheta += PROCESS_NOISE_THETA;
    }

    public void update(AprilTagDetection detection, double tagFieldX, double tagFieldY) {
        if (detection == null || detection.robotPose == null || detection.ftcPose == null) return;

        double range = detection.ftcPose.range;
        double bearingRad = Math.toRadians(detection.ftcPose.bearing);

        double measuredHeadingRad = Math.toRadians(theta); // 현재 추정 heading 기준으로 역산
        double tagRelativeX = range * Math.cos(bearingRad + measuredHeadingRad);
        double tagRelativeY = range * Math.sin(bearingRad + measuredHeadingRad);

        double measuredX = tagFieldX - tagRelativeX;
        double measuredY = tagFieldY - tagRelativeY;
        double measuredTheta = theta;

        double distanceScale = Math.max(range, 1.0) / OBSERVATION_NOISE_REFERENCE_RANGE_MM;
        double rXX = BASE_OBSERVATION_NOISE_XY * distanceScale;
        double rYY = BASE_OBSERVATION_NOISE_XY * distanceScale;
        double rThetaTheta = BASE_OBSERVATION_NOISE_THETA * distanceScale;

        double errX = measuredX - x;
        double errY = measuredY - y;
        double gateXX = pXX + rXX;
        double gateYY = pYY + rYY;
        boolean isOutlier = (errX * errX > OUTLIER_GATE_SIGMA * OUTLIER_GATE_SIGMA * gateXX)
                || (errY * errY > OUTLIER_GATE_SIGMA * OUTLIER_GATE_SIGMA * gateYY);
        if (isOutlier) return;

        double kX = pXX / (pXX + rXX);
        double kY = pYY / (pYY + rYY);
        double kTheta = pThetaTheta / (pThetaTheta + rThetaTheta);

        x += kX * errX;
        y += kY * errY;
        theta += kTheta * normalizeAngle(measuredTheta - theta);

        pXX *= (1 - kX);
        pYY *= (1 - kY);
        pThetaTheta *= (1 - kTheta);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingDeg() {
        return theta;
    }

    public double getUncertaintyX() {
        return Math.sqrt(pXX);
    }

    public double getUncertaintyY() {
        return Math.sqrt(pYY);
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
