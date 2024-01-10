package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public class Trajectories {

    private final SampleMecanumDrive drive;

    public Trajectories(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public TrajectorySequence getBlueLeft(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {

        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(12, 36))
                        .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(12, 36))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(12, 36))
                    .build();
        }

        return sequence;
    }
    






}
