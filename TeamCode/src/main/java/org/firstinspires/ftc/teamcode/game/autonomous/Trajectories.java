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

public class Trajectories {

    private final SampleMecanumDrive drive;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final PTO pto;
    private final Claw claw;

    public Trajectories(SampleMecanumDrive drive, Intake intake, Delivery delivery, V4Bar v4Bar, PTO pto, Claw claw) {
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
                    .lineTo(new Vector2d(12, 38))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineTo(new Vector2d(12, 41))
                    .lineToLinearHeading(new Pose2d(38, 35.7,Math.toRadians(190)))
                        .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(24, 44))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .lineTo(new Vector2d(24, 52))
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, 44.7,Math.toRadians(180)))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(9, 40, Math.toRadians(35)), Math.toRadians(190))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .setReversed(false)
                    .addTemporalMarker(2.5, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineTo(new Vector2d(12, 43))
                    .lineToLinearHeading(new Pose2d(38, 31, Math.toRadians(190)))
                    .build();
        }

        return sequence;
    }

    public TrajectorySequence getRedRight(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {

        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(12, -38))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineTo(new Vector2d(12, -41))
                    .lineToLinearHeading(new Pose2d(38, -35.7,Math.toRadians(180)))
                    .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.RIGHT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(24, -44))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .lineTo(new Vector2d(24, -52))
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, -44.7,Math.toRadians(180)))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(325)), Math.toRadians(190))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .setReversed(false)
                    .addTemporalMarker(2.5, ()->{
                        delivery.slideRunToPosition_Encoder(Auto.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto.CLAW_DELIVERY);
                    })
                    .lineTo(new Vector2d(12, -43))
                    .lineToLinearHeading(new Pose2d(38, -31, Math.toRadians(170)))
                    .build();
        }

        return sequence;
    }
    






}
