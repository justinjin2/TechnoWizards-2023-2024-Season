package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.teleop.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

@Autonomous(group = "Meet Three")
public class AutoBlueLeft_Meet3 extends Auto {

    private RobotState robotState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.BLUE);
        initDrive();

        Intake intake = new Intake();
        Delivery delivery = new Delivery();
        V4Bar v4Bar = new V4Bar();
        Claw claw = new Claw();
        PTO pto = new PTO();



        delivery.resetMotor(); //reset motor encoder
        intake.resetMotor();

        intake.setIntakePosition(intake.intakeInitPosition);
        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
        claw.closeBothClaw();

        RobotState robotState = RobotState.IDLE;
        
//        loopTimer = new ElapsedTime();
//        clawOpenTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(12, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

//        telemetry.addData("finalX", startPose.getX());
//        telemetry.addData("finalY", startPose.getY());
//        telemetry.addData("finalHeading", startPose.getHeading());
//        telemetry.update();

        Trajectory traj_center = drive.trajectoryBuilder(startPose)
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                //.addTemporalMarker(0, () -> {
                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
                //})
                .lineTo(new Vector2d(12, 36))
                //.splineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(-30))
                .build();

//        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                //.addTemporalMarker(0, () -> {
//                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
//                //})
//                .splineToLinearHeading(new Pose2d(19,34, Math.toRadians(140)), Math.toRadians(140))
//                .lineToConstantHeading(new Vector2d(12, 55))
//                .lineToSplineHeading(new Pose2d(48, 44, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                //.addTemporalMarker(0, () -> {
//                //    claw.clawSlideRunToPosition(claw.slideAutoHeight);
//                //})
//                .splineToLinearHeading(new Pose2d(12, 59, Math.toRadians(270)), Math.toRadians(270))
//                .splineTo(new Vector2d(6, 35), Math.toRadians(200))
//                .lineToConstantHeading(new Vector2d(23, 55))
//                .splineToLinearHeading(new Pose2d(48, 32, Math.toRadians(0)), Math.toRadians(-26))
//                .build();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();

//        startTimer();
//        loopTimer.reset();

        if (getPosition().name().equals("CENTER")) drive.followTrajectory(traj_center);
//        if (getPosition().name().equals("LEFT")) drive.followTrajectorySequence(traj_left);
//        if (getPosition().name().equals("RIGHT")) drive.followTrajectorySequence(traj_right);

        //robotState = RobotState.DELIVERY_START;

        //delivery.slideRunToPosition_Encoder(autoDeliveryPosition, delivery.slideRunHighVelocity);
        sleep(2000);

        while (!isStopRequested() && opModeIsActive()) {
//
//            loopTimer.reset();
//
//            // Will run one bulk read per cycle,
//            // because the caches are being handled manually and cleared
//            // once a loop
//            for (LynxModule hub : allHubs) {
//                hub.clearBulkCache();
//            }
//
//            switch (robotState) {
//                case DELIVERY_START:
//                   // if ((Math.abs(delivery.getMotor1Position()) + 5 > autoDeliveryPosition) ||
//                   // (Math.abs(delivery.getMotor2Position()) + 5 > autoDeliveryPosition)) {
//                   //     robotState = RobotState.DELIVERY_READY;
//                   // }
//                    break;
//                case DELIVERY_READY:
//                    //claw.openBothClaw();
//                    //robotState = RobotState.CLAW_OPEN;
//                    //clawOpenTimer.reset();
//                    break;
//                case CLAW_OPEN:
//                    //if (clawOpenTimer.milliseconds() > claw.clawOpenTime) {
//                    //    robotState = RobotState.DELIVERY_DONE;
//                    //    delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
//                    //}
//                    break;
//            }
//
//            telemetry.addData("motor1 position", delivery.getMotor1Position());
//            telemetry.addData("motor2 position", delivery.getMotor2Position());
//            telemetry.addData("Loop timer", loopTimer.milliseconds());
//            telemetry.addData("30 seconds count-down", getSecondsLeft());
//            telemetry.update();
        }

        //PoseStorage.currentPose = drive.getPoseEstimate(); //transfer pose to TeleOp
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.addData("Time Left: ", getSecondsLeft());
        telemetry.update();
    }

}
