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
public class Red_Left_Wall_19746 extends Auto_Region {

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
        claw.setClawAnglePosition(claw.clawAngleIntake);
        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        claw.closeLeftClaw();;
        claw.openRightClaw();

        robotState = RobotState.IDLE;

        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(270));
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
        sleep(2500);

        drive.followTrajectorySequence(getTrajectories().getRedLeft(getPosition(), startPose));

        if (getPosition().name().equals("LEFT")) {
            Pose2d currentPose1 = drive.getPoseEstimate();
            TrajectorySequence toIntakePosition = drive.trajectorySequenceBuilder(currentPose1)
                    .addTemporalMarker(0.5, ()->{
                        intake.setIntakePosition(intake.the5Pixel);
                    })
                    .splineToSplineHeading(new Pose2d(-50, -47, Math.toRadians(-200)), Math.toRadians(-200))
                    .addTemporalMarker(1.2, ()->{
                        intake.intakeStart();
                    })
                    .forward(7)
                    .build();
            drive.followTrajectorySequence(toIntakePosition);
        } else {
            Pose2d currentPose2 = drive.getPoseEstimate();
            TrajectorySequence toIntakePosition = drive.trajectorySequenceBuilder(currentPose2)
                    .splineToSplineHeading(new Pose2d(-50, -47, Math.toRadians(-210)), Math.toRadians(-210))
                    .addTemporalMarker(0.5, ()->{
                        intake.setIntakePosition(intake.the5Pixel);
                    })
                    .addTemporalMarker(1.2, ()->{
                        intake.intakeStart();
                    })
                    .forward(7)
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
                        if ((cycleCounterCenter == scheduledCycleCenter) && !delivery.dumpYellow) {
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_TWO+10, delivery.slideRunMediumVelocity);
                            slidePosition = Auto_Region.SLIDE_POSITION_TWO+10;
                        } else {
                            delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_SECOND_ROUND+45, delivery.slideRunHighVelocity);
                            slidePosition = Auto_Region.SLIDE_SECOND_ROUND + 45;
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
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                        delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
                        robotState = RobotState.SLIDE_ANGLE_DOWN;
                        v4BarDownTimer.reset();
                    }
                    break;
                case SLIDE_ANGLE_DOWN: //slide angle has to be down first
                    if (((Math.abs(delivery.getSlideAnglePosition()) - 10) < 0) &&
                            (v4BarDownTimer.milliseconds() > v4Bar.v4BarDownTime) &&
                            (((Math.abs(delivery.getMotor1Position()) - 10) < 0) ||
                                    (Math.abs(delivery.getMotor2Position()) - 10 < 0))) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                        robotState = RobotState.V4BAR_DOWN_MIDDLE;
                        generalTimer.reset();
                    }
                    break;
                case V4BAR_DOWN_MIDDLE:
                    if (generalTimer.milliseconds() > 200) {
                        intake.setIntakePosition(intake.intakeSafePosition);
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        claw.setClawAnglePosition(claw.clawAngleIntake);
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
                                .splineToLinearHeading(new Pose2d(48, -60, Math.toRadians(180)), Math.toRadians(-70))
                                .build();
                        drive.followTrajectorySequence(parking);
                        parked = true;
                        robotState = RobotState.IDLE;
                    }
                    break;
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
                    if (cycleCounterCenter == scheduledCycleCenter) {
                        Pose2d intakeStartPose = drive.getPoseEstimate();
                        TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakeStartPose)
                                .splineTo(new Vector2d(38, -47), Math.toRadians(-120))
                                .build();
                        drive.followTrajectorySequence(intakeStart);
                    }

                    Pose2d intakePose = drive.getPoseEstimate();
                    TrajectorySequence backOffPose = drive.trajectorySequenceBuilder(intakePose)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .splineToSplineHeading(new Pose2d(15, -58, Math.toRadians(180)), Math.toRadians(180))
                            .resetVelConstraint()
                            .splineToSplineHeading(new Pose2d(-34.5, -58, Math.toRadians(180)), Math.toRadians(180))
                            .addTemporalMarker(2.0, ()->{
                                claw.openBothClaw();
                                intake.setIntakePosition(intake.the5Pixel-0.005);
                            })
                            .splineTo(new Vector2d(-51, -47.5), Math.toRadians(-210))
                            .addTemporalMarker(2.5, ()->{
                                intake.intakeStart();
                            })
                            .build();
                    drive.followTrajectorySequence(backOffPose);
                    Pose2d intakePose1 = drive.getPoseEstimate();
                    TrajectorySequence forward = drive.trajectorySequenceBuilder(intakePose1)
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .forward(7)
                            .build();
                    drive.followTrajectorySequence(forward);
                    robotState = RobotState.INTAKE_START;
                    generalTimer.reset();
                    secondPixelTimer.reset();
                    break;
                case INTAKE_START:
                    double leftDistance = intake.getLeftPixelSensor();
                    double rightDistance = intake.getRightPixelSensor();

                    //if (cycleCounterCenter == scheduledCycleCenter) {
                    //    rightPixelOn = true;
                    //    pixelCount +=1;   //pre-load yellow pixel
                    //}

                    //try to take two instead of one

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
                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto_Region.INTAKE_TIME_OUT)) {

                        intake.setIntakePosition(intake.intakeSafePosition);

                        if (cycleCounterCenter == scheduledCycleCenter) {
                            robotState = RobotState.YELLOW_PIXEL_DELIVERY;
                            Pose2d backoffPose1 = drive.getPoseEstimate();
                            TrajectorySequence backoff = drive.trajectorySequenceBuilder(backoffPose1)
                                    .setReversed(true)
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .splineTo(new Vector2d(-34, -57), Math.toRadians(-10))
                                    .splineTo(new Vector2d(24, -57), Math.toRadians(-4))
                                    .addTemporalMarker(0.8, () -> {
                                        claw.closeBothClaw();
                                    })
                                    .addTemporalMarker(1.0, () -> {
                                        intake.intakeBackSpin();
                                    })
                                    .addTemporalMarker(1.7, () -> {
                                        intake.intakeStop();
                                        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                                    })
                                    .addTemporalMarker(1.9, () -> {
                                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                                    })
                                    .addTemporalMarker(2.1, () -> {
                                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                                    })
                                    .resetVelConstraint()
                                    .splineTo(new Vector2d(39, -55), Math.toRadians(0))
                                    .addTemporalMarker(2.3, () -> {
                                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                    })
                                    .setReversed(false)
                                    .build();
                            drive.followTrajectorySequence(backoff);
                            avoidCollisionTimer.reset();
                       } else {
                            robotState = RobotState.BACK_TO_DELIVERY;
                            Pose2d backoffPose1 = drive.getPoseEstimate();
                            TrajectorySequence backoff = drive.trajectorySequenceBuilder(backoffPose1)
                                    .setReversed(true)
                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                    .splineTo(new Vector2d(-34, -56), Math.toRadians(-10))
                                    .splineTo(new Vector2d(24, -56), Math.toRadians(-4))
                                    .addTemporalMarker(0.8, () -> {
                                        claw.closeBothClaw();
                                    })
                                    .addTemporalMarker(1.0, () -> {
                                        intake.intakeBackSpin();
                                    })
                                    .addTemporalMarker(1.7, () -> {
                                        intake.intakeStop();
                                        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                                    })
                                    .addTemporalMarker(1.9, () -> {
                                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                                    })
                                    .addTemporalMarker(2.1, () -> {
                                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY_WHITE);
                                        claw.setClawAnglePosition(Auto_Region.CLAW_SECOND_ROUND);
                                    })
                                    .resetVelConstraint()
                                    .splineTo(new Vector2d(41, -55), Math.toRadians(30))
                                    .addTemporalMarker(2.3, () -> {
                                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                    })
                                    .setReversed(false)
                                    .build();
                            drive.followTrajectorySequence(backoff);
                        }
                    }
                    break;
                case YELLOW_PIXEL_DELIVERY:
                    if (cycleCounterCenter == scheduledCycleCenter) {
                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_ROUND_ONE);
                    } else {
                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                    }

                    if ((delivery.getUltrasonicLeft() < delivery.avoidAllianceDistance) &&
                            (avoidCollisionTimer.milliseconds() < delivery.avoidCollisionTime)) {
                        robotState = RobotState.YELLOW_PIXEL_DELIVERY;
                        break;
                    } //wait for alliance move away

                    if ((delivery.getUltrasonicLeft() > delivery.avoidAllianceDistance) &&
                            (avoidCollisionTimer.milliseconds() < delivery.avoidCollisionTime)) {
                        if (getPosition().name().equals("RIGHT")) {
                            Pose2d rightPosition = drive.getPoseEstimate();
                            TrajectorySequence toRightPosition = drive.trajectorySequenceBuilder(rightPosition)
                                    .lineToLinearHeading(new Pose2d(38, -44.5, Math.toRadians(175)))
                                    .build();
                            drive.followTrajectorySequence(toRightPosition);
                        } else if (getPosition().name().equals("CENTER")) {
                            Pose2d centerPosition = drive.getPoseEstimate();
                            TrajectorySequence toCenterPosition = drive.trajectorySequenceBuilder(centerPosition)
                                    .splineToLinearHeading(new Pose2d(38, -41, Math.toRadians(180)), Math.toRadians(18))
                                    .build();
                            drive.followTrajectorySequence(toCenterPosition);
                        } else {
                            Pose2d leftPosition = drive.getPoseEstimate();
                            TrajectorySequence toLeftPosition = drive.trajectorySequenceBuilder(leftPosition)
                                    .addTemporalMarker(0, () -> {
                                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY_19746);
                                    })
                                    .lineToLinearHeading(new Pose2d(38.4, -29.1, Math.toRadians(175)))
                                    .build();
                            drive.followTrajectorySequence(toLeftPosition);
                        }
                        robotState = RobotState.BACK_TO_DELIVERY;
                        break;
                    }
                    if ((delivery.getUltrasonicLeft() < delivery.avoidAllianceDistance) &&
                            (avoidCollisionTimer.milliseconds() > delivery.avoidCollisionTime)) {
                        Pose2d dumpYellow = drive.getPoseEstimate();
                        TrajectorySequence dumpYellowPosition = drive.trajectorySequenceBuilder(dumpYellow)
                                .turn(Math.toRadians(30))
                                .build();
                        drive.followTrajectorySequence(dumpYellowPosition);
                        v4Bar.setV4BarPosition(v4Bar.v4BarYellowDump);
                        delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                        delivery.dumpYellow = true;
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

