package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

@Disabled
@Autonomous(group = "Area Championship Tournament")
public class Blue_Left_Center2x4_Simulation extends Auto_Simulation {

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.BLUE);
        initDrive();

        robotState = RobotState.IDLE;

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
                robotState = RobotState.SLIDE_DOWN;
                clawOpenTimer.reset();
            }

            switch (robotState) {
                case DELIVERY_START:
                        robotState = RobotState.CLAW_OPEN;
                    break;
                case CLAW_OPEN:
                        robotState = RobotState.SLIDE_DOWN;
                        clawOpenTimer.reset();
                    break;
                case SLIDE_DOWN:
                    if (clawOpenTimer.milliseconds() > Auto.CLAW_OPEN_TIME) {
                       robotState = RobotState.SLIDE_DOWN_HALF;
                    }
                    break;
                case SLIDE_DOWN_HALF:
                        robotState = RobotState.SLIDE_ANGLE_DOWN;
                        v4BarDownTimer.reset();
                    break;
                case SLIDE_ANGLE_DOWN: //slide angle has to be down first
                        robotState = RobotState.V4BAR_DOWN_MIDDLE;
                        generalTimer.reset();
                    break;
                case V4BAR_DOWN_MIDDLE:
                        robotState = RobotState.DELIVERY_DONE;
                        generalTimer.reset();
                    break;
                case DELIVERY_DONE:
                    if ((generalTimer.milliseconds() > 100) && (cycleCounter != 0)) {
                        leftPixelOn = false;
                        rightPixelOn = false;
                        secondPixelTimeOut = false;
                        pixelCount = 0;
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    if ((cycleCounter == 0) || (getSecondsLeft() < 4)) {
                        Pose2d parkingPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(parkingPose)
                                .lineToConstantHeading(new Vector2d(48, 16))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
                    } else {
                        robotState = RobotState.AUTO_CYCLE_START;
                    }
                    break;
            }

            switch (robotState) {
                case AUTO_CYCLE_START:
                    Pose2d intakePose = drive.getPoseEstimate();
                    TrajectorySequence intakeStart = drive.trajectorySequenceBuilder(intakePose)
                            .lineToLinearHeading(new Pose2d(36, 14, Math.toRadians(180)))
                            .splineTo(new Vector2d(-50, 14), Math.toRadians(180))
                            .build();
                    drive.followTrajectorySequence(intakeStart);
                    robotState = RobotState.INTAKE_START;
                    generalTimer.reset();
                    secondPixelTimer.reset();
                    break;
                case INTAKE_START:
                    double leftDistance = 0;
                    double rightDistance = 0;

                        leftPixelOn = true;

                        rightPixelOn = true;

                    if ((secondPixelTimer.milliseconds() > SECOND_PIXEL_TIME) && (!secondPixelTimeOut)){
                        pixelCount = pixelCount + 1;
                        secondPixelTimeOut = true;
                    }

                    if (((leftPixelOn) && (rightPixelOn)) ||
                            (generalTimer.milliseconds() > Auto.INTAKE_TIME_OUT)) {

                        robotState = RobotState.BACK_TO_DELIVERY;

                        Pose2d intakePose1 = drive.getPoseEstimate();
                        TrajectorySequence backoff = drive.trajectorySequenceBuilder(intakePose1)
                                .setReversed(true)
                                .splineTo(new Vector2d(41,22), Math.toRadians(24))
                                .build();
                        drive.followTrajectorySequence(backoff);
                    }
                    break;
                case BACK_TO_DELIVERY:
                    if (!drive.isBusy()) {
                        cycleCounter--;
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

