package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AprilTagDetector {

    private final HardwareMap hardwareMap;
    private final String cameraName = "Webcam 1";

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    public AprilTagDetector(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    /**
     * Function to get all detection bearings.
     * @return a map of tagID -> bearing
     */
    public HashMap<Integer, Double> getRawBearings() {

        HashMap<Integer, Double> finalBearings = new HashMap<>();

        for (AprilTagDetection detection : getDetections()) {
            if (detection.metadata == null) {
                continue;
            }

            finalBearings.put(detection.id, detection.ftcPose.bearing);
        }


        return finalBearings;
    }

    private List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
}
