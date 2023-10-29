package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;


@Config
@Autonomous(group = "Meet One")
public class AutoBlueLeft extends Auto{

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot


        initPropDetector(PropColor.BLUE);
        initDrive();
        telemetry.addLine("before trajectory build");
        telemetry.update();
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d(11, 66, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(17, 27), Math.toRadians(-90.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(17, 47), Math.toRadians(-90.00))
                .setReversed(false)
                .splineTo(new Vector2d (51, 35), Math.toRadians(-3))
                .build();
        drive.setPoseEstimate(sequence.start());

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();
        startTimer();

        // start doing stuff
        //drive.followTrajectorySequence(getTrajectories().getBlueLeft(getPosition()));
        drive.followTrajectorySequence(sequence);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("30 seconds count-down", 30 - (int)getTimeSeconds());
            telemetry.update();
        }


    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
