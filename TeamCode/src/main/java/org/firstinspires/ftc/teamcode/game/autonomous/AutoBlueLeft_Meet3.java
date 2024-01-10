package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropColor;

import java.util.List;

@Autonomous(group = "Meet Three")
public class AutoBlueLeft_Meet3 extends Auto {

    private RobotState robotState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        initPropDetector(PropColor.BLUE);
        initDrive();

//        Intake intake = new Intake();
//        Delivery delivery = new Delivery();
//        V4Bar v4Bar = new V4Bar();
//        Claw claw = new Claw();
//        PTO pto = new PTO();

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);


        delivery.resetMotor(); //reset motor encoder
        intake.resetMotor();

        intake.setIntakePosition(intake.intakeInitPosition);
        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
        claw.closeBothClaw();

        RobotState robotState = RobotState.IDLE;


        Pose2d startPose = new Pose2d(12, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

//        telemetry.addData("finalX", startPose.getX());
//        telemetry.addData("finalY", startPose.getY());
//        telemetry.addData("finalHeading", startPose.getHeading());
//        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();

        startTimer();


        if (getPosition().name().equals("CENTER")) drive.followTrajectorySequence(getTrajectories().getBlueLeft(getPosition(), startPose));
//        if (getPosition().name().equals("LEFT")) drive.followTrajectorySequence(traj_left);
//        if (getPosition().name().equals("RIGHT")) drive.followTrajectorySequence(traj_right);

        robotState = RobotState.DELIVERY_START;


        while (!isStopRequested() && opModeIsActive()) {
//
            loopTimer.reset();
//
            // Will run one bulk read per cycle,
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch (robotState) {
                case DELIVERY_START:
                    if (((Math.abs(delivery.getMotor1Position()) + 5) > Auto.slidePosition1) ||
                            (Math.abs(delivery.getMotor2Position()) + 5 > Auto.slidePosition1)) {
                        delivery.slideRunToPosition_Encoder(Auto.slidePosition2, delivery.slideRunHighVelocity);
                        robotState = RobotState.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if (((Math.abs(delivery.getMotor2Position()) + 5) > Auto.slidePosition2) ||
                            (Math.abs(delivery.getMotor2Position()) + 5 > Auto.slidePosition2)) {
                        claw.openBothClaw();
                        robotState = RobotState.SLIDE_DOWN;
                        clawOpenTimer.reset();
                    }
                    break;
                case SLIDE_DOWN:
                    if (clawOpenTimer.milliseconds() > Auto.clawOpenTime){
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.CLAW_ANGLE_INTAKE;
                    }
                    break;
                case CLAW_ANGLE_INTAKE:
                    if (((Math.abs(delivery.getMotor2Position()) - 150) < delivery.slideStart) ||
                            (Math.abs(delivery.getMotor2Position()) - 150 < delivery.slideStart)) {
                        claw.setClawAnglePosition(claw.clawAngleIntake);
                        robotState = RobotState.V4BAR_DOWN;
                    }
                    break;
                case V4BAR_DOWN:
                    if (((Math.abs(delivery.getMotor2Position()) - 5) > delivery.slideStart) ||
                            (Math.abs(delivery.getMotor2Position()) - 5 > delivery.slideStart)) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        Pose2d currentPose = drive.getPoseEstimate();
                        TrajectorySequence parking = drive.trajectorySequenceBuilder(currentPose)
                                .lineToConstantHeading(new Vector2d(48, 12))
                                .build();
                        drive.followTrajectorySequence(parking);
                        robotState = RobotState.IDLE;
                    }
                    break;

            }

        }

        PoseStorage.currentPose = drive.getPoseEstimate(); //transfer pose to TeleOp
    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.addData("Time Left: ", getSecondsLeft());
        telemetry.update();
    }

}

