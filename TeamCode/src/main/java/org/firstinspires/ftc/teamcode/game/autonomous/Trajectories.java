package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.partitioning.Side;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Trajectories {

    public enum AutoSide {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public enum TrajectoryPosition {
        LEFT,
        CENTER,
        RIGHT
    }



    private final SampleMecanumDrive drive;
    private final Claw claw;


    private final Map<TrajectoryPosition, TrajectorySequence> trajectorySequences = new HashMap<>();

    public Trajectories(SampleMecanumDrive drive, Claw claw) {
        this.drive = drive;
        this.claw = claw;
    }


    public void buildTrajectorySequences(AutoSide side, Pose2d startPose) {

        TrajectorySequence left = null;
        TrajectorySequence center = null;
        TrajectorySequence right = null;

        if (side.equals(AutoSide.BLUE_LEFT)) {

            center = drive.trajectorySequenceBuilder(startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .addTemporalMarker(0, () -> {
                        claw.clawSlideRunToPosition(claw.slideAutoHeight);
                    })
                    .splineTo(new Vector2d(12, 36), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(38, 28, Math.toRadians(0)), Math.toRadians(-3))
                    .build();

            left = drive.trajectorySequenceBuilder(startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .addTemporalMarker(0, () -> {
                        claw.clawSlideRunToPosition(claw.slideAutoHeight);

                    })
                    //.splineTo(new Vector2d(15, 35), Math.toRadians(-30))
                    .splineToLinearHeading(new Pose2d(19,34, Math.toRadians(-50)), Math.toRadians(-50))
                    .lineToConstantHeading(new Vector2d(12, 55))
                    .lineToSplineHeading(new Pose2d(38, 28, Math.toRadians(0)))
                    .build();

            right = drive.trajectorySequenceBuilder(startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .addTemporalMarker(0, () -> {
                        claw.clawSlideRunToPosition(claw.slideAutoHeight);
                    })
                    .splineToLinearHeading(new Pose2d(12, 59, Math.toRadians(270)), Math.toRadians(270))
                    .splineTo(new Vector2d(6, 35), Math.toRadians(200))
                    .lineToConstantHeading(new Vector2d(23, 55))
                    .splineToLinearHeading(new Pose2d(38, 28, Math.toRadians(0)), Math.toRadians(-26))
                    .build();


        }
        else if (side.equals(AutoSide.BLUE_RIGHT)) {

        }
        else if (side.equals(AutoSide.RED_LEFT)) {

        }
        else if (side.equals(AutoSide.RED_RIGHT)) {

        }

        trajectorySequences.put(TrajectoryPosition.LEFT, left);
        trajectorySequences.put(TrajectoryPosition.CENTER, center);
        trajectorySequences.put(TrajectoryPosition.RIGHT, right);
    }

    public Map<TrajectoryPosition, TrajectorySequence> getSequences() {
        return trajectorySequences;
    }





}
