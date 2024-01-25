package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

@Autonomous(group = "Meet Three")
public class AutoRedRight_Meet3 extends Auto {

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
        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
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

        drive.followTrajectorySequence(getTrajectories().getRedRight(getPosition(), startPose));

        robotState = RobotState.DELIVERY_START;

        while (!isStopRequested() && opModeIsActive()) {

            loopTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
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
                    if (generalTimer.milliseconds() > 100) {
                        Pose2d currentPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(currentPose)
                                .lineToConstantHeading(new Vector2d(48, -12))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
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

