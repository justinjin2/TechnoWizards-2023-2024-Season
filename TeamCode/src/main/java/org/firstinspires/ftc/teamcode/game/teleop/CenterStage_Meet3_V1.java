package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

import java.util.List;

@TeleOp(group = "Meet 3")
public class CenterStage_Meet3_V1 extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime loopTimer;
    ElapsedTime waitingTimer;
    ElapsedTime intakeBackSpinTimer;
    ElapsedTime v4BarUpTimer;
    ElapsedTime clawAngleTimer;
    ElapsedTime clawOpenTimer;

    private double driveSpeedRatio = 1.0;
    private RobotState robotState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        Intake intake = new Intake();
        Delivery delivery = new Delivery();
        V4Bar v4Bar = new V4Bar();
        Claw claw = new Claw();
        PTO pto = new PTO();

        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        Controllers_V1 controllers = new Controllers_V1(this, intake, delivery, v4Bar, claw, pto);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);

        intake.resetMotor();
        delivery.resetMotor();

        intake.setIntakePosition(intake.intakeInitPosition);
        sleep(400);
        claw.setClawAnglePosition(claw.clawAngleIntake);
        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        claw.openBothClaw();

        displayPoseTelemetry();

        loopTimer = new ElapsedTime();
        waitingTimer = new ElapsedTime();
        intakeBackSpinTimer = new ElapsedTime();
        v4BarUpTimer = new ElapsedTime();
        clawAngleTimer = new ElapsedTime();
        clawOpenTimer = new ElapsedTime();

        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * driveSpeedRatio,
                            -gamepad1.left_stick_x * driveSpeedRatio,
                            -gamepad1.right_stick_x * driveSpeedRatio
                    )
            );

            controllers.updateCopies(gamepad1, gamepad2);

            drive.update();

            controllers.readInputs(gamepad1, gamepad2);

            // ----------------------------------------- //
            // ---------- INTAKE FSM START ------------- //
            // ----------------------------------------- //

            switch (robotState) {
                case INTAKE_START:
                    /*
                    if (intake.getLeftPixelSensor()) claw.closeLeftClaw();
                    if (intake.getRightPixelSensor()) claw.closeRightClaw();
                    if (intake.getLeftPixelSensor() && intake.getRightPixelSensor()) {
                        robotState = RobotState.INTAKE_STOP;
                        intakeBackSpinTimer.reset();
                    }
                    */
                    intakeBackSpinTimer.reset();
                    break;
                case INTAKE_BACKSPIN:       //this also wait for claw close
                    if (intakeBackSpinTimer.milliseconds() > intake.backSpinTime) {
                        robotState = RobotState.INTAKE_STOP;
                    }
                    break;
                case INTAKE_STOP:
                    intake.intakeStop();
                    delivery.resetMotor();
                    robotState = RobotState.V4BAR_UP_STAGE1;
                    v4BarUpTimer.reset();
                    break;
                case V4BAR_UP_STAGE1:
                    if (v4BarUpTimer.milliseconds() > v4Bar.v4BarUpStage1Time) {
                    v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                    robotState = RobotState.CLAW_ANGLE_STAGE1;
                    clawAngleTimer.reset();
                    }
                    break;
                case CLAW_ANGLE_STAGE1:
                    if (clawAngleTimer.milliseconds() > claw.clawAngleStage1Time) {
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                        robotState = RobotState.INTAKE_DONE;
                        waitingTimer.reset();
                    }
                    break;
                case INTAKE_DONE:
                    if (waitingTimer.milliseconds() > 200) {
                        intake.setIntakeCenter();
                        robotState = RobotState.IDLE;
                    }
                    break;
            }

            // ----------------------------------------- //
            // ---------- DELIVERY FSM START ----------- //
            // ----------------------------------------- //

            switch (robotState) {
                case DELIVERY_START:
                    robotState = RobotState.V4BAR_UP_STAGE2;
                    waitingTimer.reset();   //waiting for intake move to safe position
                    break;
                case V4BAR_UP_STAGE2:
                    if (waitingTimer.milliseconds() > 200) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
                        robotState = RobotState.CLAW_ANGLE_STAGE2;
                        v4BarUpTimer.reset();
                    }
                    break;
                case CLAW_ANGLE_STAGE2:
                    if (waitingTimer.milliseconds() > v4Bar.v4BarUpStage2Time) {
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                        robotState = RobotState.DELIVERY_READY;
                    }
                case DELIVERY_READY:
                    if (controllers.deliveryKey == 'a') {
                        delivery.slideRunToPosition_Encoder((int) controllers.target[0][0], delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(controllers.target[0][1]);
                        claw.setClawAnglePosition(controllers.target[0][2]);
                        delivery.slideAngleRunToPosition((int)controllers.target[0][3]);
                    }

                    if (controllers.deliveryKey == 'x') {
                        delivery.slideRunToPosition_Encoder((int) controllers.target[1][0], delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(controllers.target[1][1]);
                        claw.setClawAnglePosition(controllers.target[1][2]);
                        delivery.slideAngleRunToPosition((int)controllers.target[1][3]);
                    }

                    if (controllers.deliveryKey == 'y') {
                        delivery.slideRunToPosition_Encoder((int) controllers.target[2][0], delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(controllers.target[2][1]);
                        claw.setClawAnglePosition(controllers.target[2][2]);
                        delivery.slideAngleRunToPosition((int)controllers.target[2][3]);
                    }

                    clawOpenTimer.reset();
                    break;
                case CLAW_OPEN:
                    if (clawOpenTimer.milliseconds() > claw.clawOpenTime) {
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.SLIDE_DOWN;
                    }
                    break;
                case SLIDE_DOWN:
                    if (((Math.abs(delivery.getMotor1Position()) - 180) < 0) ||
                            (Math.abs(delivery.getMotor2Position()) - 180 < 0))
                    {
                        v4Bar.setV4BarPosition(v4Bar.v4BarCenterPosition);
                        claw.setClawAnglePosition(claw.clawAngleIntake);
                        delivery.slideAngleRunToPosition(delivery.slideStart);
                        waitingTimer.reset();
                        robotState = RobotState.SLIDE_ANGLE_DOWN;
                    }
                    break;
                case SLIDE_ANGLE_DOWN: //slide angle has to be down first
                    if (((Math.abs(delivery.getSlideAnglePosition()) + 5) > 0) &&
                            (waitingTimer.milliseconds() > claw.clawAngleStage1Time)) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        robotState = RobotState.DELIVERY_DONE;
                        waitingTimer.reset();
                    }
                    break;
                case DELIVERY_DONE:
                    if (waitingTimer.milliseconds() > 400) {
                        intake.setIntakePosition(intake.intakeCenterPosition);
                        robotState = RobotState.IDLE;
                        controllers.deliveryKey = '\0';
                    }
                    break;
            }

            displayTelemetry(drive, intake, delivery, v4Bar, claw);

        }
    }

    public void displayPoseTelemetry() {
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());
    }

    public void displayTelemetry(SampleMecanumDrive drive, Intake intake, Delivery delivery, V4Bar v4Bar, Claw claw) {
        telemetry.addData("motor1 position", delivery.getMotor1Position());
        telemetry.addData("motor2 position", delivery.getMotor2Position());
        telemetry.addData("slide angle position", delivery.getSlideAnglePosition());
        telemetry.addData("v4Bar left position", v4Bar.getV4BarLeftPosition());
        telemetry.addData("v4Bar right position", v4Bar.getV4BarRightPosition());
        telemetry.addData("claw angle position", claw.getClawAngle());
        telemetry.addData("intake position", intake.getIntakeDownPosition());
        telemetry.addData("motor1 current", intake.getMotor1Current());
        telemetry.addData("motor2 current", intake.getMotor2Current());
        telemetry.addData("left pixel on", intake.getLeftPixelSensor());
        telemetry.addData("right pixel on", intake.getRightPixelSensor());
        telemetry.addData("left claw on", claw.getLeftClawSensor());
        telemetry.addData("right claw on", claw.getRightClawSensor());
        telemetry.addData("slide angle motor current", delivery.getSlideAngleMotorCurrent());
        telemetry.addData("Loop Timer", loopTimer.milliseconds());
        telemetry.update();
    }

    public void setDriveSpeedRatio(double ratio) {
        driveSpeedRatio = ratio;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public void setRobotState(RobotState state) {
        robotState = state;
    }

}
