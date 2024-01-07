package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

@Config
@Autonomous(group = "Meet Two")
public class AutoBlue_LeftFSM_Meet2 extends Auto_Meet2 {

    public enum Delivery_State {
        DELIVERY_IDLE,
        DELIVERY_START,
        DELIVERY_READY,
        ROBOT_FORWARD,
        CLAW_OPEN,
        ROBOT_BACKWARD,
        DELIVERY_DONE
    }

    ElapsedTime loopTimer;
    ElapsedTime clawOpenTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        Intake intake = new Intake();
        Delivery delivery = new Delivery();
        V4Bar v4Bar = new V4Bar();
        Claw claw = new Claw();

        initPropDetector(PropColor.BLUE);
        initDrive();
        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);

        intake.resetMotor();
        delivery.resetMotor(); //reset motor encoder
        claw.setClawAngleCenter();
        v4Bar.setV4BarInit();

        Delivery_State delivery_state = Delivery_State.DELIVERY_IDLE;
        loopTimer = new ElapsedTime();
        clawOpenTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(-90.00));
        drive.setPoseEstimate(startPose);
        telemetry.addData("finalX", startPose.getX());
        telemetry.addData("finalY", startPose.getY());
        telemetry.addData("finalHeading", startPose.getHeading());
        telemetry.update();


        TrajectorySequence traj_center = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.addTemporalMarker(0, () -> {
                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
                //})
                .splineTo(new Vector2d(12, 36), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(48, 38, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.addTemporalMarker(0, () -> {
                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
                //})
                .splineToLinearHeading(new Pose2d(19,34, Math.toRadians(-50)), Math.toRadians(-50))
                .lineToConstantHeading(new Vector2d(12, 55))
                .lineToSplineHeading(new Pose2d(48, 44, Math.toRadians(0)))
                .build();

        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.addTemporalMarker(0, () -> {
                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
                //})
                .splineToLinearHeading(new Pose2d(12, 59, Math.toRadians(270)), Math.toRadians(270))
                .splineTo(new Vector2d(6, 35), Math.toRadians(200))
                .lineToConstantHeading(new Vector2d(23, 55))
                .splineToLinearHeading(new Pose2d(48, 32, Math.toRadians(0)), Math.toRadians(-26))
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

            loopTimer.reset();

            switch (delivery_state) {
                case DELIVERY_START:
                    delivery.slideRunToTarget_PID(autoDeliveryPosition);
                    if ((Math.abs(delivery.getMotor1Position()) + 5) > autoDeliveryPosition) {
                        delivery_state = Delivery_State.DELIVERY_READY;
                    }
                    break;
                case DELIVERY_READY:
                    delivery.slideRunToTarget_PID(autoDeliveryPosition);
                    claw.openBothClaw();
                    delivery_state = Delivery_State.CLAW_OPEN;
                    clawOpenTimer.reset();
                    break;
                case CLAW_OPEN:
                    delivery.slideRunToTarget_PID(autoDeliveryPosition);
                    if (clawOpenTimer.milliseconds() > claw.clawOpenTime) {
                        delivery_state = Delivery_State.DELIVERY_DONE;
                        delivery.slideRunToPosition_Encoder(slideHomePosition, delivery.slideRunLowVelocity);
                    }
                    break;
            }

            telemetry.addData("motor1 position", delivery.getMotor1Position());
            telemetry.addData("motor2 position", delivery.getMotor2Position());
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
