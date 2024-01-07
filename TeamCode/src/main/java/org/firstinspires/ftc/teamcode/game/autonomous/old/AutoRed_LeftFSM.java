package org.firstinspires.ftc.teamcode.game.autonomous.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.autonomous.Auto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;


@Config
@Autonomous(group = "Meet One")
public class AutoRed_LeftFSM extends Auto {

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

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot

        initPropDetector(PropColor.RED);
        initDrive();

        clawOld.closeArm();
        clawOld.wristUp();
        clawOld.droneClose();

        Delivery_State delivery_state = Delivery_State.DELIVERY_IDLE;
        loopTimer = new ElapsedTime();
        armOpenTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90.00));
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
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-52, -48), Math.toRadians(90))
                .addTemporalMarker(4, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideStart);
                })
                .splineToLinearHeading(new Pose2d(-45, -12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(44, -12, Math.toRadians(3)), Math.toRadians(3))
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideAutoHeight);
                })
                .splineToLinearHeading(new Pose2d(-42, -39, Math.toRadians(130)), Math.toRadians(150))
                .lineToLinearHeading(new Pose2d(-36, -46, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(4, () -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideStart);
                })
                .lineToConstantHeading(new Vector2d(-36, -12))
                .lineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(44, -12, Math.toRadians(3)), Math.toRadians(3))
                .build();

        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                .addTemporalMarker(0, () -> {
//                    claw.clawSlideRunToPosition(claw.slideAutoHeight);
//
//                })
                .splineToLinearHeading(new Pose2d(-39, -50, Math.toRadians(45)), Math.toRadians(30))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-32, -37))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(-52, -48), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-45, -12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(44, -12, Math.toRadians(3)), Math.toRadians(3))
                .build();

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();

        startTimer();
        loopTimer.reset();

        if (getPosition().name().equals("CENTER")) {
            drive.followTrajectorySequence(traj_center);
            Pose2d currentCenterPose = drive.getPoseEstimate();
            TrajectorySequence traj_center2 = drive.trajectorySequenceBuilder(currentCenterPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(15, 12))
                    .addTemporalMarker(1, () -> {
                        clawOld.clawSlideRunToPosition(clawOld.slideAutoHeight);
                    })
                    .splineToLinearHeading(new Pose2d(40, 38, Math.toRadians(0)), Math.toRadians(-3))
                    .build();
            //drive.followTrajectorySequence(traj_center2);
        }
        if (getPosition().name().equals("LEFT")) {
            drive.followTrajectorySequence(traj_left);
            Pose2d currentLeftPose = drive.getPoseEstimate();
            TrajectorySequence traj_left2 = drive.trajectorySequenceBuilder(currentLeftPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .forward(5)
                    .build();
            //drive.followTrajectorySequence(traj_left2);
        }
        if (getPosition().name().equals("RIGHT")) {
            drive.followTrajectorySequence(traj_right);
            Pose2d currentRightPose = drive.getPoseEstimate();
            TrajectorySequence traj_right2 = drive.trajectorySequenceBuilder(currentRightPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .forward(5)
                    .build();
            //drive.followTrajectorySequence(traj_right2);
        }

        //delivery_state = Delivery_State.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {

            loopTimer.reset();

            switch (delivery_state) {
                case DELIVERY_START:
                    Pose2d currentPose = new Pose2d(48, 38, Math.toRadians(0));
                    //Pose2d currentPose1 = drive.getPoseEstimate();
                    TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .forward(1)
                            .build();
                    drive.followTrajectorySequence(forward);
                    delivery_state = Delivery_State.ROBOT_FORWARD;
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
                        drive.followTrajectorySequence(backward);
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
                    drive.followTrajectorySequence(parking);
                    delivery_state = Delivery_State.DELIVERY_IDLE;
                    break;
            }

            telemetry.addData("Loop timer", loopTimer.milliseconds());
            telemetry.addData("30 seconds count-down", getSecondsLeft());
            telemetry.update();
        }
        PoseStorage.currentPose = drive.getPoseEstimate(); //transfer pose to TeleOp
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
