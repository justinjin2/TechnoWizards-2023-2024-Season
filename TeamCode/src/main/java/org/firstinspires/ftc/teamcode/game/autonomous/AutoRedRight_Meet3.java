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


        // TODO: drive.followTrajectorySequence(getTrajectories().getBlueLeft(getPosition(), startPose));

        robotState = RobotState.DELIVERY_START;


        while (!isStopRequested() && opModeIsActive()) {

            loopTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch (robotState) {
                case DELIVERY_START:
                    if (((Math.abs(delivery.getMotor1Position()) + 5) > Auto.SLIDE_POSITION_ONE) ||
                            (Math.abs(delivery.getMotor2Position()) + 5 > Auto.SLIDE_POSITION_ONE)) {
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                        robotState = RobotState.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if (((Math.abs(delivery.getMotor2Position()) + 5) > Auto.SLIDE_POSITION_TWO) ||
                            (Math.abs(delivery.getMotor2Position()) + 5 > Auto.SLIDE_POSITION_TWO)) {
                        claw.openBothClaw();
                        robotState = RobotState.SLIDE_DOWN;
                        clawOpenTimer.reset();
                    }
                    break;
                case SLIDE_DOWN:
                    if (clawOpenTimer.milliseconds() > Auto.CLAW_OPEN_TIME){
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.CLAW_ANGLE_INTAKE;
                    }
                    break;
                case CLAW_ANGLE_INTAKE:
                    if (((Math.abs(delivery.getMotor2Position()) - 150) < delivery.slideStart) ||
                            (Math.abs(delivery.getMotor2Position()) - 150 < delivery.slideStart)) {
                        claw.setClawAnglePosition(claw.clawAngleIntake);
                        robotState = RobotState.V4BAR_DOWN;
                    }
                    break;
                case V4BAR_DOWN:
                    if (((Math.abs(delivery.getMotor2Position()) - 5) > delivery.slideStart) ||
                            (Math.abs(delivery.getMotor2Position()) - 5 > delivery.slideStart)) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        Pose2d currentPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(currentPose)
                                .lineToConstantHeading(new Vector2d(48, 12))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
                    }
                    break;

            }

        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.addData("Time Left: ", getSecondsLeft());
        telemetry.update();
    }

}

