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


        if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER) {
            blueRight = drive.trajectorySequenceBuilder(new Pose2d(-71.61, -37.68, Math.toRadians(6.17)))
                    .splineTo(new Vector2d(-47.93, -32.94), Math.toRadians(4.24))
                    .splineTo(new Vector2d(-28.80, -31.96), Math.toRadians(0.00))
                    .splineTo(new Vector2d(-46.16, -34.52), Math.toRadians(189.46))
                    .setReversed(true)
                    .splineTo(new Vector2d(-59.97, -38.27), Math.toRadians(191.45))
                    .splineTo(new Vector2d(-40.24, -53.65), Math.toRadians(-37.95))
                    .setReversed(false)
                    .splineTo(new Vector2d(-24.26, -57.21), Math.toRadians(9.03))
                    .splineTo(new Vector2d(-3.35, -53.06), Math.toRadians(72.76))
                    .splineTo(new Vector2d(-1.78, 15.78), Math.toRadians(95.19))
                    .splineTo(new Vector2d(-14.40, 33.14), Math.toRadians(148.24))
                    .splineTo(new Vector2d(-31.96, 40.44), Math.toRadians(105.95))
                    .build();
            drive.setPoseEstimate(blueRight.start());
        }
        else if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT) {
            blueRight = drive.trajectorySequenceBuilder(new Pose2d(-71.61, -37.68, Math.toRadians(6.17)))
                    .splineTo(new Vector2d(-47.93, -32.94), Math.toRadians(4.24))
                    .splineTo(new Vector2d(-28.80, -31.96), Math.toRadians(0.00))
                    .splineTo(new Vector2d(-46.16, -34.52), Math.toRadians(189.46))
                    .setReversed(true)
                    .splineTo(new Vector2d(-59.97, -38.27), Math.toRadians(191.45))
                    .splineTo(new Vector2d(-40.24, -53.65), Math.toRadians(-37.95))
                    .setReversed(false)
                    .splineTo(new Vector2d(-24.26, -57.21), Math.toRadians(9.03))
                    .splineTo(new Vector2d(-3.35, -53.06), Math.toRadians(72.76))
                    .splineTo(new Vector2d(-1.78, 15.78), Math.toRadians(95.19))
                    .splineTo(new Vector2d(-14.40, 33.14), Math.toRadians(148.24))
                    .splineTo(new Vector2d(-31.96, 40.44), Math.toRadians(105.95))
                    .build();
            drive.setPoseEstimate(blueRight.start());
        }
        else {
            blueRight = drive.trajectorySequenceBuilder(new Pose2d(-71.61, -37.68, Math.toRadians(6.17)))
                    .splineTo(new Vector2d(-47.93, -32.94), Math.toRadians(4.24))
                    .splineTo(new Vector2d(-28.80, -31.96), Math.toRadians(0.00))
                    .splineTo(new Vector2d(-46.16, -34.52), Math.toRadians(189.46))
                    .setReversed(true)
                    .splineTo(new Vector2d(-59.97, -38.27), Math.toRadians(191.45))
                    .splineTo(new Vector2d(-40.24, -53.65), Math.toRadians(-37.95))
                    .setReversed(false)
                    .splineTo(new Vector2d(-24.26, -57.21), Math.toRadians(9.03))
                    .splineTo(new Vector2d(-3.35, -53.06), Math.toRadians(72.76))
                    .splineTo(new Vector2d(-1.78, 15.78), Math.toRadians(95.19))
                    .splineTo(new Vector2d(-14.40, 33.14), Math.toRadians(148.24))
                    .splineTo(new Vector2d(-31.96, 40.44), Math.toRadians(105.95))
                    .build();
            drive.setPoseEstimate(blueRight.start());
        }


        
        return blueRight;
    }






}
