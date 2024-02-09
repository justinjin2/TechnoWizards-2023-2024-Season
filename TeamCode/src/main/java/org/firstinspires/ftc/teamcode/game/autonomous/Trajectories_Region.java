package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public class Trajectories_Region {

    private final SampleMecanumDrive drive;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final PTO pto;
    private final Claw claw;

    public Trajectories_Region(SampleMecanumDrive drive, Intake intake, Delivery delivery, V4Bar v4Bar, PTO pto, Claw claw) {
        this.drive = drive;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.pto = pto;
        this.claw = claw;
    }

    public TrajectorySequence getBlueLeft(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {

        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(20,42, Math.toRadians(75)), Math.toRadians(75))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(-18))
                    .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(24, 44))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .lineTo(new Vector2d(26, 58))
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, 42,Math.toRadians(180)))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(10, 41, Math.toRadians(35)), Math.toRadians(190))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .setReversed(false)
                    .lineTo(new Vector2d(12, 43))
                    .addTemporalMarker(2.5, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, 28, Math.toRadians(180)))
                    .build();
        }

        return sequence;
    }

    public TrajectorySequence getBlueRight(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {

        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-41,41, Math.toRadians(120)), Math.toRadians(120))
                    .setReversed(false)
                    .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(-32,40), Math.toRadians(-50))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(-43, 46), Math.toRadians(245))
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-36, 51, Math.toRadians(90)), Math.toRadians(90))
                    .build();
        }

        return sequence;
    }

    public TrajectorySequence getRedRight(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {

        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(12, -38, Math.toRadians(285)))
                    .lineTo(new Vector2d(12, -43))
                    .lineToLinearHeading(new Pose2d(38, -35,Math.toRadians(180)))
                    .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.RIGHT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(24, -44))
                    .lineTo(new Vector2d(26, -58))
                    .lineToLinearHeading(new Pose2d(38, -42,Math.toRadians(180)))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(325)), Math.toRadians(190))
                    .setReversed(false)
                    .lineTo(new Vector2d(12, -43))
                    .lineToLinearHeading(new Pose2d(38, -28, Math.toRadians(180)))
                    .build();
        }

        return sequence;
    }
    






}
