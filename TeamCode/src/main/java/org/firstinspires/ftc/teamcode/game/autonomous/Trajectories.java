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

    public TrajectorySequence getBlueRight(TeamPropDetector.TSEDetectorPipeline.TSEPosition position){

        TrajectorySequence blueRight = null;


        // Needs drive.trajectorySequenceBuilder
        if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER) {
            blueRight = null;


            drive.setPoseEstimate(blueRight.start());
        }
        else if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT) {
            blueRight = null;


            drive.setPoseEstimate(blueRight.start());
        }
        else {
            blueRight = null;


            drive.setPoseEstimate(blueRight.start());
        }


        
        return blueRight;
    }






}
