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

    public TrajectorySequence getBlueLeft(TeamPropDetector.TSEDetectorPipeline.TSEPosition position){

        TrajectorySequence blueLeft = null;


        // Needs drive.trajectorySequenceBuilder
        if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER) {
            blueLeft = drive.trajectorySequenceBuilder(new Pose2d(11.84, 66.28, Math.toRadians(-90.00)))
                    .splineTo(new Vector2d(17.16, 27.02), Math.toRadians(-90.00))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(17.36, 47.93), Math.toRadians(-90.00))
                    .setReversed(false)
                    .splineTo(new Vector2d (51.29, 35.70), Math.toRadians(-3.58))
                    .build();

            drive.setPoseEstimate(blueLeft.start());
        }
        else if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT) {
            blueLeft = null;


            drive.setPoseEstimate(blueLeft.start());
        }
        else {
            blueLeft = null;


            drive.setPoseEstimate(blueLeft.start());
        }


        
        return blueLeft;
    }






}
