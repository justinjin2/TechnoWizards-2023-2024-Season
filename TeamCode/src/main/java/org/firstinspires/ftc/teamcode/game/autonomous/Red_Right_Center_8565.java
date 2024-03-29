package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

//@Disabled
@Autonomous(group = "Area Championship Tournament")
public class Red_Right_Center_8565 extends Auto_Region {

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.RED);
        initDrive();

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);

        delivery.resetMotor();
        intake.resetMotor();

        intake.setIntakePosition(intake.intakeInitPosition);
        claw.setClawAnglePosition(claw.clawAngleInit);
        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
        claw.closeBothClaw();

        robotState = RobotState.IDLE;

        Pose2d startPose = new Pose2d(12, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();

        startTimer();

        drive.followTrajectorySequence(getTrajectories().getRedRight_8565(getPosition(), startPose));

        robotState = RobotState.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();

            // Will run one bulk read per cycle,
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if ((getSecondsLeft() < 2) && (!cycleTimeOut)) { //time to park
                robotState = RobotState.DELIVERY_DONE;
                cycleCounterCenter = 0;
                cycleTimeOut = true;
            }

            if (parked) robotState = RobotState.IDLE;

            switch (robotState) {
                case DELIVERY_START:
                    if (((Math.abs(delivery.getMotor1Position()) + 15) > Auto_Region.SLIDE_POSITION_ONE) ||
                            (Math.abs(delivery.getMotor2Position()) + 15 > Auto_Region.SLIDE_POSITION_ONE)) {
                        if (cycleCounterCenter == scheduledCycleCenter) {
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_TWO+10, delivery.slideRunMediumVelocity);
                            slidePosition = Auto_Region.SLIDE_POSITION_TWO+10;
                        } else {
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_SECOND_ROUND+35, delivery.slideRunMediumVelocity);
                            slidePosition = Auto_Region.SLIDE_SECOND_ROUND+35;
                        }
                        generalTimer.reset();
                        robotState = RobotState.DELIVERY_READY;
                    }
                    break;
                case DELIVERY_READY:
                    if (((Math.abs(delivery.getMotor1Position()) + 10) > slidePosition) ||
                            (Math.abs(delivery.getMotor2Position()) + 10 > slidePosition) ||
                            (generalTimer.milliseconds() > 800)) {
                        claw.openBothClaw();
                        robotState = RobotState.CLAW_OPEN;
                        clawOpenTimer.reset();
                    }
                    break;
                case CLAW_OPEN:
                    if (clawOpenTimer.milliseconds() > claw.clawOpenTime+100) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
                        robotState=RobotState.V4BAR_UP;
                        generalTimer.reset();
                    }
                    break;
                case V4BAR_UP:
                    if (generalTimer.milliseconds() > 150) {
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.SLIDE_DOWN;
                    }
                    break;
                case SLIDE_DOWN:
                    if (((Math.abs(delivery.getMotor1Position()) - 250) < 0) ||
                            (Math.abs(delivery.getMotor2Position()) - 250 < 0))
                    {
                        Pose2d prepareIntake = drive.getPoseEstimate();
                        TrajectorySequence  awayBackDrop = drive.trajectorySequenceBuilder(prepareIntake)
                            .lineToLinearHeading(new Pose2d(36, -15, Math.toRadians(180)))
                                .addTemporalMarker(0, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                                    delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                                })
                                .addTemporalMarker(0.5, ()->{
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                                })
                                .addTemporalMarker(0.8, ()->{
                                    intake.setIntakePosition(intake.intakeCenterPosition);
                                    v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                                    claw.setClawAnglePosition(claw.clawAngleIntake);
                                })
                            .build();
                        drive.followTrajectorySequence(awayBackDrop);
                        robotState = RobotState.DELIVERY_DONE;
                        generalTimer.reset();
                    }
                    break;
                case DELIVERY_DONE:
                    if ((generalTimer.milliseconds() > 100) && (cycleCounterCenter > 0)) {
                        leftPixelOn = false;
                        rightPixelOn = false;
                        secondPixelTimeOut = false;
                        pixelCount = 0;
                        if (cycleCounterCenter < scheduledCycleCenter) intake.the5Pixel -= 0.03; //not the first time
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    if (cycleCounterCenter == 0) {
                        intake.setIntakePosition(intake.intakeInitPosition);
                        Pose2d parkingPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(parkingPose)
                                .lineToConstantHeading(new Vector2d(48, -16))
                                .build();
                        drive.followTrajectorySequence(parking);
                        parked = true;
                        robotState = RobotState.IDLE;
                    }
                    break;
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
                    Pose2d intakePose = drive.getPoseEstimate();
                    TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakePose)
                            .splineToLinearHeading(new Pose2d(-47, -12, Math.toRadians(180)), Math.toRadians(180))
                            .addTemporalMarker(2, ()->{
                                claw.openBothClaw();
                                intake.setIntakePosition(intake.the5Pixel);
                            })
                            .addTemporalMarker(2.5, ()->{
                                intake.intakeStart();
                            })
                            .build();
                    drive.followTrajectorySequence(intakeStart);

                    //using ultrasound sensor to calculate distance to wall
                    if (!drive.isBusy()) {
                        double leftSideDistance = intake.getUltrasonicBackLeft();
                        double rightSideDistance = intake.getUltrasonicBackRight();
                        if ((leftSideDistance < 0) && (rightSideDistance > 0)) leftSideDistance = rightSideDistance;
                        if ((leftSideDistance > 0) && (rightSideDistance < 0)) rightSideDistance = leftSideDistance;
                        if ((leftSideDistance < 0) && (rightSideDistance < 0)) {
                            leftSideDistance = 15;
                            rightSideDistance = 15;
                        }
                        double diff = leftSideDistance - rightSideDistance;
                        if (Math.abs(diff) < 1.0) {
                            double forwardDistance = ((leftSideDistance + rightSideDistance) / 2) - delivery.intakeExtendLength;
                            Pose2d intakePose1 = drive.getPoseEstimate();
                            TrajectorySequence forward = drive.trajectorySequenceBuilder(intakePose1)
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .forward((int) Math.round(forwardDistance))
                                    .build();
                            drive.followTrajectorySequence(forward);
                        } else {
                            double min = Math.min(leftSideDistance, rightSideDistance) - delivery.intakeExtendLength;
                            Pose2d intakePose1 = drive.getPoseEstimate();
                            TrajectorySequence forward = drive.trajectorySequenceBuilder(intakePose1)
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .forward((int) Math.round(min))
                                    .build();
                            drive.followTrajectorySequence(forward);
                        }
                    }
                    robotState = RobotState.INTAKE_START;
                    generalTimer.reset();
                    secondPixelTimer.reset();
                    break;
                case INTAKE_START:
                    double leftDistance = intake.getLeftPixelSensor();
                    double rightDistance = intake.getRightPixelSensor();

                    if ((leftDistance < intake.leftPixelDetectDistance) && (!leftPixelOn)) {
                        claw.closeLeftClaw();
                        pixelCount = pixelCount + 1;
                        if (pixelCount < 2) intake.setIntakePositionStep(intake.theNextPixel);
                        leftPixelOn = true;
                        }
                    if ((rightDistance < intake.rightPixelDetectDistance) && (!rightPixelOn)) {
                        claw.closeRightClaw();
                        pixelCount = pixelCount + 1;
                        if (pixelCount < 2) intake.setIntakePositionStep(intake.theNextPixel);
                        rightPixelOn = true;
                        }
                    if ((secondPixelTimer.milliseconds() > Auto_Region.SECOND_PIXEL_TIME) &&
                            (pixelCount < 2)) {
                        intake.setIntakePositionStep(intake.theNextPixel);
                        pixelCount = pixelCount + 1;
                        secondPixelTimer.reset();
                        }
                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto_Region.INTAKE_TIME_OUT)) {
                        robotState = RobotState.BACK_TO_DELIVERY;
                        intake.setIntakePosition(intake.intakeSafePosition);

                        Pose2d deliveryPose= drive.getPoseEstimate();
                        TrajectorySequence backoff = drive.trajectorySequenceBuilder(deliveryPose)
                                .setReversed(true)
                                .splineTo(new Vector2d(39.5,-22), Math.toRadians(-22))
                                .addTemporalMarker(0.8, ()->{
                                    claw.closeBothClaw();
                                })
                                .addTemporalMarker(1.0, ()->{
                                    intake.intakeBackSpin();
                                })
                                .addTemporalMarker(1.3, ()->{
                                    intake.intakeStop();
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                                })
                                .addTemporalMarker(1.4, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                                })
                                .addTemporalMarker(1.6, ()->{
                                    v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY_WHITE);
                                    claw.setClawAnglePosition(Auto_Region.CLAW_SECOND_ROUND);
                                })
                                .addTemporalMarker(2, ()->{
                                    delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                                    delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                })
                                .build();
                        drive.followTrajectorySequence(backoff);
                        }
                    break;
                case BACK_TO_DELIVERY:
                    if (!drive.isBusy()) {
                        cycleCounterCenter--;
                        robotState = RobotState.DELIVERY_START;
                        }
                    break;
            }

            telemetry.addData("loop timer", loopTimer.milliseconds());
            telemetry.addData("time left", getSecondsLeft());
            telemetry.update();
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}

