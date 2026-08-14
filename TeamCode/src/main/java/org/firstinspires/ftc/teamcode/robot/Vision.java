package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.Comparator;
import java.util.List;

public class Vision {

    public enum ArtifactColor {
        PURPLE,
        GREEN
    }

    public enum Motif {
        GPP, // ID 21
        PGP, // ID 22
        PPG  // ID 23
    }

    public static final int TAG_ID_GOAL_BLUE = 20;
    public static final int TAG_ID_GOAL_RED = 24;
    public static final int TAG_ID_OBELISK_1 = 21;
    public static final int TAG_ID_OBELISK_2 = 22;
    public static final int TAG_ID_OBELISK_3 = 23;

    private static final int TAG_CAM_WIDTH = 1280;
    private static final int TAG_CAM_HEIGHT = 720;
    private static final double CAM_FX = 1385.31;
    private static final double CAM_FY = 1385.31;
    private static final double CAM_CX = 973.922;
    private static final double CAM_CY = 554.487;

    private static final int ARTIFACT_CAM_WIDTH = 640;
    private static final int ARTIFACT_CAM_HEIGHT = 480;

    private static final int BLOB_BLUR_SIZE = 5;
    private static final int BLOB_DILATE_SIZE = 15;
    private static final int BLOB_ERODE_SIZE = 15;
    private static final double BLOB_MIN_AREA = 50;
    private static final double BLOB_MAX_AREA = 20000;
    private static final double BLOB_MIN_CIRCULARITY = 0.6;
    private static final double BLOB_MAX_CIRCULARITY = 1.0;

    private final VisionPortal tagVisionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private final VisionPortal artifactVisionPortal;
    private final ColorBlobLocatorProcessor purpleBlobProcessor;
    private final ColorBlobLocatorProcessor greenBlobProcessor;

    public Vision(HardwareMap hardwareMap, String tagWebcamName, String artifactWebcamName,
                  Position cameraPosition, YawPitchRollAngles cameraOrientation) {

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(CAM_FX, CAM_FY, CAM_CX, CAM_CY)
                .build();

        tagVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, tagWebcamName))
                .setCameraResolution(new Size(TAG_CAM_WIDTH, TAG_CAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(viewIds[0])
                .addProcessor(aprilTagProcessor)
                .build();

        purpleBlobProcessor = buildColorBlobProcessor(ColorRange.ARTIFACT_PURPLE);
        greenBlobProcessor = buildColorBlobProcessor(ColorRange.ARTIFACT_GREEN);

        artifactVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, artifactWebcamName))
                .setCameraResolution(new Size(ARTIFACT_CAM_WIDTH, ARTIFACT_CAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewIds[1])
                .addProcessor(purpleBlobProcessor)
                .addProcessor(greenBlobProcessor)
                .build();
    }

    private static ColorBlobLocatorProcessor buildColorBlobProcessor(ColorRange colorRange) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(BLOB_BLUR_SIZE)
                .setDilateSize(BLOB_DILATE_SIZE)
                .setErodeSize(BLOB_ERODE_SIZE)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    public List<AprilTagDetection> getAllDetections() {
        return aprilTagProcessor.getDetections();
    }

    public AprilTagDetection getDetectionById(int tagId) {
        for (AprilTagDetection detection : getAllDetections()) {
            if (detection.metadata != null && detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getGoalTagDetection(Alliance alliance) {
        return getDetectionById(alliance == Alliance.RED ? TAG_ID_GOAL_RED : TAG_ID_GOAL_BLUE);
    }

    public AprilTagDetection getObeliskDetection() {
        AprilTagDetection detection = getDetectionById(TAG_ID_OBELISK_1);
        if (detection != null) return detection;
        detection = getDetectionById(TAG_ID_OBELISK_2);
        if (detection != null) return detection;
        return getDetectionById(TAG_ID_OBELISK_3);
    }

    public Motif getTargetMotif() {
        AprilTagDetection detection = getObeliskDetection();
        if (detection == null) return null;

        if (detection.id == TAG_ID_OBELISK_1) return Motif.GPP;
        if (detection.id == TAG_ID_OBELISK_2) return Motif.PGP;
        if (detection.id == TAG_ID_OBELISK_3) return Motif.PPG;

        return null;
    }

    public static final ArtifactColor[] PATTERN_GPP = {
            ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
            ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
            ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE
    };
    public static final ArtifactColor[] PATTERN_PGP = {
            ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE,
            ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE,
            ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE
    };
    public static final ArtifactColor[] PATTERN_PPG = {
            ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN,
            ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN,
            ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN
    };

    public static ArtifactColor[] getPatternForMotif(Motif motif) {
        switch (motif) {
            case GPP: return PATTERN_GPP;
            case PGP: return PATTERN_PGP;
            case PPG: return PATTERN_PPG;
            default: return null;
        }
    }

    public double getRangeToTag(int tagId) {
        AprilTagDetection detection = getDetectionById(tagId);
        if (detection == null || detection.ftcPose == null) return -1;
        return detection.ftcPose.range;
    }

    public Position getRobotFieldPosition(int tagId) {
        AprilTagDetection detection = getDetectionById(tagId);
        if (detection == null || detection.robotPose == null) return null;
        return detection.robotPose.getPosition();
    }

    public List<ColorBlobLocatorProcessor.Blob> getArtifactBlobs(ArtifactColor color) {
        ColorBlobLocatorProcessor processor =
                (color == ArtifactColor.PURPLE) ? purpleBlobProcessor : greenBlobProcessor;

        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                BLOB_MIN_AREA, BLOB_MAX_AREA, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                BLOB_MIN_CIRCULARITY, BLOB_MAX_CIRCULARITY, blobs);

        return blobs;
    }

    public ColorBlobLocatorProcessor.Blob getLargestArtifactBlob(ArtifactColor color) {
        List<ColorBlobLocatorProcessor.Blob> blobs = getArtifactBlobs(color);
        if (blobs.isEmpty()) return null;

        return blobs.stream()
                .max(Comparator.comparingDouble(ColorBlobLocatorProcessor.Blob::getContourArea))
                .orElse(null);
    }

    public ColorBlobLocatorProcessor.Blob getLargestArtifactBlobAnyColor() {
        ColorBlobLocatorProcessor.Blob purple = getLargestArtifactBlob(ArtifactColor.PURPLE);
        ColorBlobLocatorProcessor.Blob green = getLargestArtifactBlob(ArtifactColor.GREEN);

        if (purple == null) return green;
        if (green == null) return purple;
        return (purple.getContourArea() >= green.getContourArea()) ? purple : green;
    }

    public double getBlobNormalizedX(ColorBlobLocatorProcessor.Blob blob) {
        double centerX = blob.getBoxFit().boundingRect().x + blob.getBoxFit().boundingRect().width / 2.0;
        return (centerX / ARTIFACT_CAM_WIDTH) * 2.0 - 1.0;
    }

    public void close() {
        tagVisionPortal.close();
        artifactVisionPortal.close();
    }
}