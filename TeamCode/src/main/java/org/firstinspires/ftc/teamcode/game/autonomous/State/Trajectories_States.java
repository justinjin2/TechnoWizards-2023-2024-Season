package org.firstinspires.ftc.teamcode.game.autonomous.State;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.game.autonomous.Auto_Region;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public class Trajectories_States {

    private final SampleMecanumDrive drive;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final PTO pto;
    private final Claw claw;

    public Trajectories_States(SampleMecanumDrive drive, Intake intake, Delivery delivery, V4Bar v4Bar, PTO pto, Claw claw) {
        this.drive = drive;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.pto = pto;
        this.claw = claw;
    }

    public TrajectorySequence getRedRight2_6(TeamPropDetector.TSEDetectorPipeline.TSEPosition position, Pose2d startPose) {



        TrajectorySequence sequence;

        if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.CENTER)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(20,-41, Math.toRadians(285)), Math.toRadians(285))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .addTemporalMarker(1.5, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_States.SLIDE_POSITION_TWO, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(37.5, -36, Math.toRadians(180)), Math.toRadians(18))
                    .build();
        }
        else if (position.equals(TeamPropDetector.TSEDetectorPipeline.TSEPosition.LEFT)) {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(9, -41, Math.toRadians(325)), Math.toRadians(190))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .setReversed(false)
                    .lineTo(new Vector2d(12, -43))
                    .addTemporalMarker(2.5, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, -28, Math.toRadians(180)))
                    .build();
        }
        else {
            sequence = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(24, -45))
                    .addTemporalMarker(1, ()->{
                        intake.setIntakePosition(intake.intakeSafePosition);
                    })
                    .lineTo(new Vector2d(29, -59))
                    .addTemporalMarker(2, ()->{
                        delivery.slideRunToPosition_Encoder(Auto_Region.SLIDE_POSITION_ONE, delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
                        claw.setClawAnglePosition(Auto_Region.CLAW_DELIVERY);
                    })
                    .lineToLinearHeading(new Pose2d(38, -42,Math.toRadians(180)))
                    .build();
        }

        return sequence;
    }
}
