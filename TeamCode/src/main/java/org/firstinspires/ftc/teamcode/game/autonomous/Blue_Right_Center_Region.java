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
public class Blue_Right_Center_Region extends Auto_Region {

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.BLUE);
        initDrive();

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);

        delivery.resetMotor();
        intake.resetMotor();

        intake.setIntakePosition(intake.intakeInitPosition);
        claw.setClawAnglePosition(claw.clawAngleIntake);
        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        claw.closeLeftClaw();
        claw.openRightClaw();

        robotState = RobotState.IDLE;

        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(90));
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

        drive.followTrajectorySequence(getTrajectories().getBlueRight(getPosition(), startPose));

        if (getPosition().name().equals("RIGHT")) {
            Pose2d currentPose1 = drive.getPoseEstimate();
            TrajectorySequence toIntakePosition = drive.trajectorySequenceBuilder(currentPose1)
                    .setReversed(true)
                    .splineTo(new Vector2d(-36, 14), Math.toRadians(-90))
                    .addTemporalMarker(0.5, ()->{
                        intake.setIntakePosition(intake.the5Pixel);
                    })
                    .splineToSplineHeading(new Pose2d(-47, 15, Math.toRadians(180)), Math.toRadians(180))
                    .addTemporalMarker(1.2, ()->{
                        intake.intakeStart();
                    })
                    .forward(8)
                    .build();
            drive.followTrajectorySequence(toIntakePosition);
        } else {
            Pose2d currentPose2 = drive.getPoseEstimate();
            TrajectorySequence toIntakePosition = drive.trajectorySequenceBuilder(currentPose2)
                    .splineToSplineHeading(new Pose2d(-47, 15, Math.toRadians(180)), Math.toRadians(30))
                    .addTemporalMarker(0.5, ()->{
                        intake.setIntakePosition(intake.the5Pixel);
                    })
                    .addTemporalMarker(1.2, ()->{
                        intake.intakeStart();
                    })
                    .forward(8)
                    .build();
            drive.followTrajectorySequence(toIntakePosition);
        }

        robotState = RobotState.INTAKE_START;
        generalTimer.reset();
        secondPixelTimer.reset();

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
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                            slidePosition = Auto_Region.SLIDE_POSITION_TWO;
                        } else {
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_SECOND_ROUND, delivery.slideRunHighVelocity);
                            slidePosition = Auto_Region.SLIDE_SECOND_ROUND;
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
                        cycleCounterCenter -=1;  //cycle counter reduce
                        robotState = RobotState.CLAW_OPEN;
                        clawOpenTimer.reset();
                    }
                    break;
                case CLAW_OPEN:
                    if (clawOpenTimer.milliseconds() > claw.clawOpenTime) {
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
                                .lineToLinearHeading(new Pose2d(36, 15, Math.toRadians(180)))
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
                        if (cycleCounterCenter < scheduledCycleCenter) intake.the5Pixel -= 0.015; //not the first time
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    if (cycleCounterCenter == 0) {
                        intake.setIntakePosition(intake.intakeInitPosition);
                        Pose2d parkingPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(parkingPose)
                                .lineToConstantHeading(new Vector2d(48, 16))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
                    }
                    break;
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
                    Pose2d intakePose = drive.getPoseEstimate();
                    TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakePose)
                            .splineToLinearHeading(new Pose2d(-47, 15, Math.toRadians(175)), Math.toRadians(180))
                            .addTemporalMarker(2, () -> {
                                claw.openBothClaw();
                                intake.setIntakePosition(intake.the5Pixel);
                            })
                            .addTemporalMarker(2.5, () -> {
                                intake.intakeStart();
                            })
                            .build();
                    drive.followTrajectorySequence(intakeStart);
                    Pose2d intakePose1 = drive.getPoseEstimate();
                    TrajectorySequence forward = drive.trajectorySequenceBuilder(intakePose1)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .forward(8)
                            .build();
                    drive.followTrajectorySequence(forward);
                    robotState = RobotState.INTAKE_START;
                    generalTimer.reset();
                    secondPixelTimer.reset();
                    break;
                case INTAKE_START:
                    double leftDistance = intake.getLeftPixelSensor();
                    double rightDistance = intake.getRightPixelSensor();

                    if (cycleCounterCenter == scheduledCycleCenter) {
                        leftPixelOn = true;
                        pixelCount +=1;   //pre-load yellow pixel
                    }

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
                    if ((secondPixelTimer.milliseconds() > SECOND_PIXEL_TIME) &&
                            (pixelCount < 2)) {
                        intake.setIntakePositionStep(intake.theNextPixel);
                        pixelCount = pixelCount + 1;
                        secondPixelTimer.reset();
                    }
                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto_Region.INTAKE_TIME_OUT)) {

                        if (cycleCounterCenter == scheduledCycleCenter) {
                            robotState = RobotState.YELLOW_PIXEL_DELIVERY;
                        } else {
                            robotState = RobotState.BACK_TO_DELIVERY;
                        }
                        intake.setIntakePosition(intake.intakeSafePosition);

                        Pose2d deliveryPose= drive.getPoseEstimate();
                        TrajectorySequence backoff = drive.trajectorySequenceBuilder(deliveryPose)
                                .setReversed(true)
                                .splineTo(new Vector2d(42,22), Math.toRadians(22))
                                .addTemporalMarker(0.8, ()->{
                                    claw.closeBothClaw();
                                })
                                .addTemporalMarker(1.0, ()->{
                                    intake.intakeBackSpin();
                                })
                                .addTemporalMarker(1.7, ()->{
                                    intake.intakeStop();
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                                })
                                .addTemporalMarker(1.9, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                                })
                                .addTemporalMarker(2.1, ()->{
                                    v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                                    claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                                })
                                .addTemporalMarker(2.3, ()->{
                                    delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                                    delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                })
                                .build();
                        drive.followTrajectorySequence(backoff);
                    }
                    break;
                case YELLOW_PIXEL_DELIVERY:
                    if (cycleCounterCenter == scheduledCycleCenter) {
                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_ROUND_ONE);
                    } else {
                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                    }

                    if (getPosition().name().equals("LEFT")) {
                        Pose2d leftPosition = drive.getPoseEstimate();
                        TrajectorySequence toLeftPosition = drive.trajectorySequenceBuilder(leftPosition)
                                .lineToLinearHeading(new Pose2d(38, 42,Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(toLeftPosition);
                    } else if (getPosition().name().equals("CENTER")) {
                        Pose2d centerPosition = drive.getPoseEstimate();
                        TrajectorySequence toCenterPosition = drive.trajectorySequenceBuilder(centerPosition)
                                .splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(-18))
                                .build();
                        drive.followTrajectorySequence(toCenterPosition);
                    } else {
                        Pose2d rightPosition = drive.getPoseEstimate();
                        TrajectorySequence toRightPosition = drive.trajectorySequenceBuilder(rightPosition)
                                .lineToLinearHeading(new Pose2d(38, 28, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(toRightPosition);

                    }
                    robotState = RobotState.BACK_TO_DELIVERY;
                    break;
                case BACK_TO_DELIVERY:
                    if (!drive.isBusy()) {
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

