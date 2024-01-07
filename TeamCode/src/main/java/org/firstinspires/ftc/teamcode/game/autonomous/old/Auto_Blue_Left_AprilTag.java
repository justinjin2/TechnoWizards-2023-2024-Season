package org.firstinspires.ftc.teamcode.game.autonomous.old;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.game.autonomous.Auto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Config
@Autonomous(group = "Meet One")
public class Auto_Blue_Left_AprilTag extends Auto {

    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)

    public enum Delivery_State {
        DELIVERY_IDLE,
        DELIVERY_START,
        ROBOT_FORWARD,
        CLAW_OPEN,
        ROBOT_BACKWARD,
        DELIVERY_DONE
    }

    ElapsedTime loopTimer;
    ElapsedTime armOpenTimer;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot

        initPropDetector(PropColor.BLUE);
        initDrive();

        clawOld.closeArm();
        clawOld.wristUp();

        Delivery_State delivery_state = Delivery_State.DELIVERY_IDLE;
        loopTimer = new ElapsedTime();
        armOpenTimer = new ElapsedTime();

        int backboard_tag_id = -1;
        double  rangeError = 0;
        double  headingError = 0;
        double  yawError = 0;

        double speed = 0, turn = 0, strafe = 0;


        Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(-90.00));
        drive.setPoseEstimate(startPose);
        telemetry.addData("finalX", startPose.getX());
        telemetry.addData("finalY", startPose.getY());
        telemetry.addData("finalHeading", startPose.getHeading());
        telemetry.update();


        TrajectorySequence traj_center = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideAutoHeight);
                })
                .splineTo(new Vector2d(12, 36), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38, 28, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideAutoHeight);

                })
                //.splineTo(new Vector2d(15, 35), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(19,34, Math.toRadians(-50)), Math.toRadians(-50))
                .lineToConstantHeading(new Vector2d(12, 55))
                .lineToSplineHeading(new Pose2d(38, 28, Math.toRadians(0)))
                .build();

        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideAutoHeight);
                })
                .splineToLinearHeading(new Pose2d(12, 59, Math.toRadians(270)), Math.toRadians(270))
                .splineTo(new Vector2d(6, 35), Math.toRadians(200))
                .lineToConstantHeading(new Vector2d(23, 55))
                .splineToLinearHeading(new Pose2d(38, 28, Math.toRadians(0)), Math.toRadians(-26))
                .build();

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
                //.setLensIntrinsics()
                //.setNumThreads()
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getSampleTagLibrary())

                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        startTimer();
        loopTimer.reset();

        if (getPosition().name().equals("CENTER")) {
            //drive.followTrajectorySequence(traj_center);
            backboard_tag_id = 2;
        }
        if (getPosition().name().equals("LEFT")) {
            //drive.followTrajectorySequence(traj_left);
            backboard_tag_id = 1;
        }
        if (getPosition().name().equals("RIGHT")) {
            //drive.followTrajectorySequence(traj_right);
            backboard_tag_id = 3;
        }

        // Make sure camera is streaming before we process AprilTag
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
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
                if ((detection.id == backboard_tag_id)) {
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
                    delivery_state = Delivery_State.ROBOT_BACKWARD;

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

        delivery_state = Delivery_State.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {

            loopTimer.reset();

            switch (delivery_state) {
                case DELIVERY_START:
                    if (backboard_tag_id == -1) { //No AprilTag found
                        Pose2d currentPose1 = drive.getPoseEstimate();
                        TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose1)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .forward(15)
                                .build();
                        //drive.followTrajectorySequence(forward);
                        delivery_state = Delivery_State.ROBOT_FORWARD;
                    }
                    break;
                case ROBOT_FORWARD:
                    if (!drive.isBusy()) {
                        clawOld.openArm();
                        armOpenTimer.reset();
                        delivery_state = Delivery_State.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if ((armOpenTimer.milliseconds() > clawOld.armOpenTime)) {
                        Pose2d currentPose2 = drive.getPoseEstimate();
                        TrajectorySequence backward = drive.trajectorySequenceBuilder(currentPose2)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(4)
                                .build();
                        //drive.followTrajectorySequence(backward);
                        delivery_state = Delivery_State.ROBOT_BACKWARD;
                    }
                    break;
                case ROBOT_BACKWARD:
                    if (!drive.isBusy()) {
                        clawOld.clawSlideRunToPosition(clawOld.slideStart);
                        clawOld.openArm();
                        clawOld.wristUp();
                        delivery_state = Delivery_State.DELIVERY_DONE;
                    }
                    break;
                case DELIVERY_DONE:
                    Pose2d currentPose3 = drive.getPoseEstimate();
                    TrajectorySequence parking = drive.trajectorySequenceBuilder(currentPose3)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .lineToConstantHeading(new Vector2d(48, 12))
                            .build();
                    //drive.followTrajectorySequence(parking);
                    delivery_state = Delivery_State.DELIVERY_IDLE;
                    break;
            }

            telemetry.addData("Loop timer", loopTimer.milliseconds());
            telemetry.addData("30 seconds count-down", getSecondsLeft());
            telemetry.update();
        }
    }

    private void moveToAprilTag(double x, double y, double yaw)
    {
        //calculate target location and drive to
    }
    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}