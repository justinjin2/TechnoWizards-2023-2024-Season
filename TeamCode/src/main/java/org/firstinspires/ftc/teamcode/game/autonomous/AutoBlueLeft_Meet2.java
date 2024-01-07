package org.firstinspires.ftc.teamcode.game.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(group = "Meet Two")
public class AutoBlueLeft_Meet2 extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.BLUE);
        initDrive();


        claw.closeArm();
        claw.wristUp();

        DeliveryState deliveryState = DeliveryState.DELIVERY_IDLE;

        Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(-90.00));
        drive.setPoseEstimate(startPose);

        getTrajectories().buildTrajectorySequences(Trajectories.AutoSide.BLUE_LEFT, startPose);

        telemetry.addData("finalX", startPose.getX());
        telemetry.addData("finalY", startPose.getY());
        telemetry.addData("finalHeading", startPose.getHeading());
        telemetry.update();


        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }


        getPropDetector().closeWebcam();

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        setVisionPortal(new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build()
        );

        startTimer();
        getLoopTimer().reset();

        if (getPosition().name().equals("CENTER")) {

            // drive.followTrajectorySequence(getTrajectories().getSequences().get(Trajectories.TrajectoryPosition.CENTER));
            backboardTagID = 2;
        }
        if (getPosition().name().equals("LEFT")) {
            // drive.followTrajectorySequence(getTrajectories().getSequences().get(Trajectories.TrajectoryPosition.LEFT));
            backboardTagID = 1;
        }
        if (getPosition().name().equals("RIGHT")) {
            // drive.followTrajectorySequence(getTrajectories().getSequences().get(Trajectories.TrajectoryPosition.RIGHT));
            backboardTagID = 3;
        }

        // Make sure camera is streaming before we process AprilTag
        if (getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        sleep(2000); //maybe deleted if start using trajectory
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag
                if ((detection.id == backboardTagID)) {
                    // Yes, we want to use this tag.
                    telemetry.addData("Found", "ID %d (%s)", detection.id, detection.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
                    telemetry.update();
                    rangeError = (detection.ftcPose.range - DESIRED_DISTANCE);
                    headingError = detection.ftcPose.bearing;
                    yawError = detection.ftcPose.yaw;
                    moveToAprilTag(rangeError, headingError, yawError);
                    deliveryState = DeliveryState.ROBOT_BACKWARD;

                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    telemetry.update();
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                telemetry.update();
            }
        }

        deliveryState = DeliveryState.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {

            getLoopTimer().reset();

            switch (deliveryState) {
                case DELIVERY_START:
                    if (backboardTagID == -1) { //No AprilTag found
                        Pose2d currentPose1 = drive.getPoseEstimate();
                        TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(15)
                                .build();
                        //drive.followTrajectorySequence(forward);
                        deliveryState = DeliveryState.ROBOT_FORWARD;
                    }
                    break;
                case ROBOT_FORWARD:
                    if (!drive.isBusy()) {
                        claw.openArm();
                        getAutoTimer().reset();
                        deliveryState = DeliveryState.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if ((getAutoTimer().milliseconds() > claw.armOpenTime)) {
                        Pose2d currentPose2 = drive.getPoseEstimate();
                        TrajectorySequence backward = drive.trajectorySequenceBuilder(currentPose2)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(4)
                                .build();
                        //drive.followTrajectorySequence(backward);
                        deliveryState = DeliveryState.ROBOT_BACKWARD;
                    }
                    break;
                case ROBOT_BACKWARD:
                    if (!drive.isBusy()) {
                        claw.clawSlideRunToPosition(claw.slideStart);
                        claw.openArm();
                        claw.wristUp();
                        deliveryState = DeliveryState.DELIVERY_DONE;
                    }
                    break;
                case DELIVERY_DONE:
                    Pose2d currentPose3 = drive.getPoseEstimate();
                    TrajectorySequence parking = drive.trajectorySequenceBuilder(currentPose3)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .lineToConstantHeading(new Vector2d(48, 12))
                            .build();
                    //drive.followTrajectorySequence(parking);
                    deliveryState = DeliveryState.DELIVERY_IDLE;
                    break;
            }


            updateTelemetry();
        }
    }

    private void moveToAprilTag(double x, double y, double yaw)
    {
        //calculate target location and drive to
    }

    public void updateTelemetry() {
        telemetry.addData("Loop timer", getLoopTimer().milliseconds());
        telemetry.addData("30 seconds count-down", getSecondsLeft());
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
