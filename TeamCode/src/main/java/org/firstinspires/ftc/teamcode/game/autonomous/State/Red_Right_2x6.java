package org.firstinspires.ftc.teamcode.game.autonomous.State;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.game.autonomous.State.Auto_States;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

//@Disabled
@Autonomous(group = "Area Championship Tournament")
public class Red_Right_2x6 extends Auto_States {

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

        drive.followTrajectorySequence(getTrajectories().getRedRight2_6(getPosition(), startPose));
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
                    if (((Math.abs(delivery.getMotor1Position()) + 15) > Auto_States.SLIDE_POSITION_TWO) ||
                            (Math.abs(delivery.getMotor2Position()) + 15 > Auto_States.SLIDE_POSITION_TWO)) {
                        if (cycleCounterCenter == scheduledCycleCenter) {
                            slidePosition = Auto_States.SLIDE_POSITION_TWO;
                        } else {
                            slidePosition = Auto_States.SLIDE_POSITION_TWO;
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
                    if (cycleCounterCenter == 0) {
                        Pose2d parkingPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(parkingPose)
                                .lineToConstantHeading(new Vector2d(48, -16))
                                .addTemporalMarker(0, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                                    delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                                })
                                .addTemporalMarker(0.5, ()->{
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                                })
                        .addTemporalMarker(0.5, ()->{
                            intake.setIntakePosition(intake.intakeInitPosition);
                        })
                                .build();
                        drive.followTrajectorySequence(parking);
                        parked = true;
                        robotState = RobotState.IDLE;
                    }
                    else if (cycleCounterCenter >1) {
                        Pose2d prepareIntake = drive.getPoseEstimate();
                        TrajectorySequence  awayBackDrop = drive.trajectorySequenceBuilder(prepareIntake)
                                .splineTo(new Vector2d(11.00, -9.00), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-47, -13), Math.toRadians(180))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .splineToConstantHeading(new Vector2d(-54, -13), Math.toRadians(180))
                                .addTemporalMarker(0, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                                    delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                                })
                                .addTemporalMarker(0.5, ()->{
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                                })
                                .addTemporalMarker(0.8, ()->{
                                    intake.setIntakePosition(intake.the5Pixel);
                                    v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                                    claw.setClawAnglePosition(claw.clawAngleIntake);
                                })
                                .addTemporalMarker(1, ()->{
                                    claw.openBothClaw();
                                })
                                .addTemporalMarker(1.2, ()->{
                                    intake.intakeStart();
                                })
                            .build();
                        drive.followTrajectorySequence(awayBackDrop);
                        robotState = RobotState.DELIVERY_DONE;
                        generalTimer.reset();
                    }
                    else if ((cycleCounterCenter == 1)){
                        Pose2d prepareIntake2 = drive.getPoseEstimate();
                        TrajectorySequence  awayBackDrop2 = drive.trajectorySequenceBuilder(prepareIntake2)
                                .splineTo(new Vector2d(11, -9), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-47, -13), Math.toRadians(180))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                                .splineToConstantHeading(new Vector2d(-54, -24), Math.toRadians(180))
                                .addTemporalMarker(0, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                                    delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                                })
                                .addTemporalMarker(0.5, ()->{
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                                })
                                .addTemporalMarker(0.8, ()->{
                                    intake.setIntakePosition(intake.the5Pixel);
                                    v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                                    claw.setClawAnglePosition(claw.clawAngleIntake);
                                })
                                .addTemporalMarker(1, ()->{
                                claw.openBothClaw();
                                    intake.setIntakePosition(intake.the5Pixel);
                            })
                            .addTemporalMarker(1.3, ()->{
                                intake.intakeStart();
                            })
                                .build();
                        drive.followTrajectorySequence(awayBackDrop2);
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
                        if (cycleCounterCenter == 3 ) intake.the5Pixel -= 0.06;
                        else if (cycleCounterCenter == 2){
                            intake.the5Pixel = 0.27; //third cycle
                        }



                        robotState = RobotState.AUTO_CYCLE_START;
                    }
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
//                    Pose2d intakePose = drive.getPoseEstimate();
//                    TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakePose)
//                            .splineToLinearHeading(new Pose2d(-47, -13, Math.toRadians(180)), Math.toRadians(180))
//                            .addTemporalMarker(2, ()->{
//                                claw.openBothClaw();
//                                intake.setIntakePosition(intake.the5Pixel);
//                            })
//                            .addTemporalMarker(2.5, ()->{
//                                intake.intakeStart();
//                            })
//                            .build();
//                    drive.followTrajectorySequence(intakeStart);
//                    Pose2d intakePose1 = drive.getPoseEstimate();
//                    TrajectorySequence forward = drive.trajectorySequenceBuilder(intakePose1)
//                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                        .forward(8)
//                        .build();
//                    drive.followTrajectorySequence(forward);
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
                    if ((secondPixelTimer.milliseconds() > SECOND_PIXEL_TIME) &&
                            (pixelCount < 2)) {
                        intake.setIntakePositionStep(intake.theNextPixel);
                        pixelCount = pixelCount + 1;
                        secondPixelTimer.reset();
                        }
                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto_States.INTAKE_TIME_OUT)) {

                       if (cycleCounterCenter > 1) {
                           robotState = RobotState.BACK_TO_DELIVERY;
                           intake.setIntakePosition(intake.intakeSafePosition);
                           Pose2d deliveryPose= drive.getPoseEstimate();
                        TrajectorySequence backoff = drive.trajectorySequenceBuilder(deliveryPose)
                                .setReversed(true)
                                .splineTo(new Vector2d(41,-22), Math.toRadians(-22))
                                .addTemporalMarker(1, ()->{
                                    claw.closeBothClaw();
                                })
                                .addTemporalMarker(1.2, ()->{
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
                                    v4Bar.setV4BarPosition(Auto_States.V4BAR_DELIVERY);
                                    claw.setClawAnglePosition(Auto_States.CLAW_SECOND_ROUND);
                                })
                                .addTemporalMarker(2.3, ()->{
                                    delivery.slideRunToPosition_Encoder(Auto_States.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                                    delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                })
                                .build();
                        drive.followTrajectorySequence(backoff);
                        }
                    else if(cycleCounterCenter == 1) {
                           robotState = RobotState.BACK_TO_DELIVERY;
                           intake.setIntakePosition(intake.intakeSafePosition);
                           Pose2d deliveryPose2 = drive.getPoseEstimate();
                           TrajectorySequence backoff2 = drive.trajectorySequenceBuilder(deliveryPose2)
                                   .setReversed(true)
                                   .splineTo(new Vector2d(-5, -9), Math.toRadians(-32))
                                   .splineTo(new Vector2d(40.7,-22), Math.toRadians(-22))
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
                                       v4Bar.setV4BarPosition(Auto_States.V4BAR_DELIVERY);
                                       claw.setClawAnglePosition(Auto_States.CLAW_SECOND_ROUND);
                                   })
                                   .addTemporalMarker(1.7, ()->{
                                       delivery.slideRunToPosition_Encoder(Auto_States.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                                       delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                   })
                                   .build();
                           drive.followTrajectorySequence(backoff2);
                       }

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
