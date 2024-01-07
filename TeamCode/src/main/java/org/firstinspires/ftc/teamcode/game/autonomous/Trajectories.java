package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw_Meet1;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public class Trajectories {

    private final SampleMecanumDrive drive;
    private final Claw_Meet1 clawOld;

    TrajectorySequence blueLeftCenter;




    public Trajectories(SampleMecanumDrive drive, Claw_Meet1 clawOld) {
        this.drive = drive;
        this.clawOld = clawOld;

        blueLeftCenter = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, Math.toRadians(-90.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0,() -> {
                    clawOld.clawSlideRunToPosition(clawOld.slideLow);
                })
                .splineTo(new Vector2d(12, 35.5), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90.00))
                .splineToLinearHeading(new Pose2d(48, 37, Math.toRadians(0)), Math.toRadians(-3))
                .build();
    }
    


    public TrajectorySequence getBlueLeft(TeamPropDetector.TSEDetectorPipeline.TSEPosition position){

        if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER) {
            drive.setPoseEstimate(blueLeftCenter.start());
            return blueLeftCenter;
        }
        else if (position == TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT) {

            //TODO: Change This
            drive.setPoseEstimate(blueLeftCenter.start());
            return blueLeftCenter;

        }
        else {
            //TODO: Change This
            drive.setPoseEstimate(blueLeftCenter.start());
            return blueLeftCenter;
        }

    }






}
