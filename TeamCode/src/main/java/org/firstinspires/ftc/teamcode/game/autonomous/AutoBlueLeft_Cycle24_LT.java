package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;
@Disabled
@Autonomous(group = "League Tournament")
public class AutoBlueLeft_Cycle24_LT extends Auto {

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
        claw.setClawAnglePosition(claw.clawAngleInit);
        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
        claw.closeBothClaw();

        robotState = RobotState.IDLE;
        boolean leftPixelOn = false;
        boolean rightPixelOn = false;
        boolean secondPixelTimeOut = false;
        int cycleCounter = 0;
        int pixelCount = 0;

        Pose2d startPose = new Pose2d(12, 64, Math.toRadians(90));
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

        drive.followTrajectorySequence(getTrajectories().getBlueLeft(getPosition(), startPose));

        robotState = RobotState.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();

            // Will run one bulk read per cycle,
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (getSecondsLeft() < 5) { //time to park
                claw.openBothClaw();
                robotState = RobotState.SLIDE_DOWN;
                clawOpenTimer.reset();
            }

            switch (robotState) {
                case DELIVERY_START:
                    if (((Math.abs(delivery.getMotor1Position()) + 15) > Auto.SLIDE_POSITION_ONE) ||
                            (Math.abs(delivery.getMotor2Position()) + 15 > Auto.SLIDE_POSITION_ONE)) {
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                        generalTimer.reset();
                        robotState = RobotState.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if (((Math.abs(delivery.getMotor1Position()) + 5) > Auto.SLIDE_POSITION_TWO) ||
                            (Math.abs(delivery.getMotor2Position()) + 5 > Auto.SLIDE_POSITION_TWO) ||
                            (generalTimer.milliseconds() > 1000)) {
                        claw.openBothClaw();
                        robotState = RobotState.SLIDE_DOWN;
                        clawOpenTimer.reset();
                    }
                    break;
                case SLIDE_DOWN:
                    if (clawOpenTimer.milliseconds() > Auto.CLAW_OPEN_TIME) {
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.SLIDE_DOWN_HALF;
                    }
                    break;
                case SLIDE_DOWN_HALF:
                    if (((Math.abs(delivery.getMotor1Position()) - 250) < 0) ||
                            (Math.abs(delivery.getMotor2Position()) - 250 < 0)) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                        delivery.slideAngleRunToPosition(delivery.slideStart);
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
                    if (generalTimer.milliseconds() > 250) {
                        intake.setIntakePosition(intake.intakeInitPosition);
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        claw.setClawAnglePosition(claw.clawAngleIntake);
                        robotState = RobotState.DELIVERY_DONE;
                        generalTimer.reset();
                    }
                    break;
                case DELIVERY_DONE:
                    if ((generalTimer.milliseconds() > 100) && (cycleCounter == 0)) {
                        leftPixelOn = false;
                        rightPixelOn = false;
                        secondPixelTimeOut = false;
                        pixelCount = 0;
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    if ((cycleCounter > 1) || (getSecondsLeft() < 4)) {
                        Pose2d parkingPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(parkingPose)
                                .lineToConstantHeading(new Vector2d(48, 12))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
                    } else if (cycleCounter > 0) {
                        intake.the5Pixel -= 0.03;
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    break;
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
                    Pose2d intakePose = drive.getPoseEstimate();
                    TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakePose)
                            .lineToLinearHeading(new Pose2d(36, 15, Math.toRadians(180)))
                            .splineTo(new Vector2d(-46, 15), Math.toRadians(170))
                            .addTemporalMarker(2.5, ()->{
                                claw.openBothClaw();
                                intake.intakeStart();
                                intake.setIntakePosition(intake.the5Pixel);
                            })
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .forward(9.5)
                            .build();
                    drive.followTrajectorySequence(intakeStart);
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

                    if ((secondPixelTimer.milliseconds() > SECOND_PIXEL_TIME) && (!secondPixelTimeOut)){
                        if (pixelCount == 0) intake.setIntakePositionStep(intake.theNextPixel);
                        pixelCount = pixelCount + 1;
                        secondPixelTimeOut = true;
                    }

                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto.INTAKE_TIME_OUT)) {
                        intake.setIntakePosition(intake.intakeSafePosition);
                        robotState = RobotState.BACK_TO_DELIVERY;
                        claw.closeBothClaw();
                        Pose2d intakePose1 = drive.getPoseEstimate();
                        TrajectorySequence backoff = drive.trajectorySequenceBuilder(intakePose1)
                                .lineToConstantHeading(new Vector2d(34, 15))
                                .addTemporalMarker(0.5, ()->{
                                    intake.intakeBackSpin();
                                })
                                .addTemporalMarker(1, ()->{
                                    intake.intakeStop();
                                })
                                .addTemporalMarker(1.3, ()->{
                                    v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                                })
                                .addTemporalMarker(1.5, ()->{
                                    claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                                })
                                .addTemporalMarker(1.8, ()->{
                                    v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                                    claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                                })
                                .addTemporalMarker(2, ()->{
                                    delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                                    delivery.slideAngleRunToPosition(SLIDE_ANGLE_POSITION);
                                })
                                .build();
                        drive.followTrajectorySequence(backoff);
                    }
                    break;
                case BACK_TO_DELIVERY:
                    if (!drive.isBusy()) {
                        Pose2d deliveryPose = drive.getPoseEstimate();
                        TrajectorySequence deliveryStart = drive.trajectorySequenceBuilder(deliveryPose)
                                .lineToLinearHeading(new Pose2d(39, 32, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequence(deliveryStart);
                        cycleCounter++;
                        robotState = RobotState.DELIVERY_START;
                    }
                    break;
            }

            telemetry.addData("loop timer", loopTimer.milliseconds());
            telemetry.addData("time left", getSecondsLeft());
            //telemetry.addData("motor1 current", intake.getMotor1Current());
            //telemetry.addData("motor2 current", intake.getMotor2Current());
            telemetry.update();
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}

