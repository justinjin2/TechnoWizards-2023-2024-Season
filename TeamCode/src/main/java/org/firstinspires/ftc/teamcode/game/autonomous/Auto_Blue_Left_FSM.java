package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;


@Config
@Autonomous(group = "Meet One")
public class Auto_Blue_Left_FSM extends Auto {

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

        initPropDetector(PropColor.BLUE);
        initDrive();

        claw.closeArm();
        claw.wristUp();

        Delivery_State delivery_state = Delivery_State.DELIVERY_IDLE;
        loopTimer = new ElapsedTime();
        armOpenTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(-90.00));
        drive.setPoseEstimate(startPose);
        telemetry.addData("finalX", startPose.getX());
        telemetry.addData("finalY", startPose.getY());
        telemetry.addData("finalHeading", startPose.getHeading());
        telemetry.update();


        TrajectorySequence traj_center = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    claw.clawSlideRunToPosition(claw.slideLow);
                })
                .splineTo(new Vector2d(12, 35.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(48, 37, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    claw.clawSlideRunToPosition(claw.slideLow);
                })
                .splineTo(new Vector2d(20, 30), Math.toRadians(-80))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-80))
                .splineToLinearHeading(new Pose2d(48, 42, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, () -> {
                    claw.clawSlideRunToPosition(claw.slideLow);
                })
                .splineTo(new Vector2d(22, 30), Math.toRadians(-100))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(48, 32, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();

        startTimer();
        loopTimer.reset();

        if (getPosition().name().equals("CENTER")) drive.followTrajectorySequence(traj_center);
        if (getPosition().name().equals("LEFT")) drive.followTrajectorySequence(traj_left);
        if (getPosition().name().equals("RIGHT")) drive.followTrajectorySequence(traj_right);

        delivery_state = Delivery_State.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {

            switch (delivery_state) {
                case DELIVERY_START:
                    Pose2d currentPose1 = drive.getPoseEstimate();
                    TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose1)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .forward(3)
                            .build();
                    drive.followTrajectorySequence(forward);
                    delivery_state = Delivery_State.ROBOT_FORWARD;
                    break;
                case ROBOT_FORWARD:
                    if (!drive.isBusy()) {
                        claw.openArm();
                        armOpenTimer.reset();
                        delivery_state = Delivery_State.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if ((armOpenTimer.milliseconds() > claw.armOpenTime)) {
                        Pose2d currentPose2 = drive.getPoseEstimate();
                        TrajectorySequence backward = drive.trajectorySequenceBuilder(currentPose2)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .back(3)
                                .build();
                        drive.followTrajectorySequence(backward);
                        delivery_state = Delivery_State.ROBOT_BACKWARD;
                    }
                    break;
                case ROBOT_BACKWARD:
                    if (!drive.isBusy()) {
                        claw.clawSlideRunToPosition(claw.slideStart);
                        claw.openArm();
                        claw.wristUp();
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
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
