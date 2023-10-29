package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

        claw.closeArm();
        claw.wristUp();

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, Math.toRadians(-90.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0,() -> {
                    claw.clawSlideRunToPosition(claw.slideLow);
                })
                .splineTo(new Vector2d(12, 35.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(48, 37, Math.toRadians(0)), Math.toRadians(-3))
                .build();

        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(sequence.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(51.1, 37))
                .addTemporalMarker(1, ()-> {
                    claw.openArm();
                })
                .build();

        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(sequence1.end())
                .lineToConstantHeading(new Vector2d(48, 37))
                .lineToConstantHeading(new Vector2d(48, 12))
                .addTemporalMarker(4,() -> {
                    claw.clawSlideRunToPosition(claw.slideStart);
                })
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
        drive.followTrajectorySequence(sequence1);
        drive.followTrajectorySequence(sequence2);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("30 seconds count-down", getSecondsLeft());
            telemetry.update();
        }


    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
