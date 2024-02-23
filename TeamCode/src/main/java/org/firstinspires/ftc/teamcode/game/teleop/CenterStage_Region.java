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

@TeleOp(group = "Area Championship Tournament")
public class CenterStage_Region extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime loopTimer;
    ElapsedTime waitingTimer;
    ElapsedTime intakeBackSpinTimer;
    ElapsedTime v4BarUpTimer;
    ElapsedTime v4BarDownTimer;
    ElapsedTime clawAngleTimer;
    ElapsedTime clawOpenTimer;
    ElapsedTime clawCloseTimer;
    ElapsedTime droneLaunchTimer;

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

        Controllers_Region controllers = new Controllers_Region(this, intake, delivery, v4Bar, claw, pto);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);

        displayPoseTelemetry();

        loopTimer = new ElapsedTime();
        waitingTimer = new ElapsedTime();
        intakeBackSpinTimer = new ElapsedTime();
        v4BarUpTimer = new ElapsedTime();
        v4BarDownTimer = new ElapsedTime();
        clawAngleTimer = new ElapsedTime();
        clawOpenTimer = new ElapsedTime();
        clawCloseTimer = new ElapsedTime();
        droneLaunchTimer = new ElapsedTime();

        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        if ((delivery.getSlideAngleSensor()) &&
                (delivery.getLeftSlideSensor() || delivery.getRightSlideSensor())) {
            intake.setIntakePosition(intake.intakeSafePosition);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            sleep(300);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
            claw.openBothClaw();
        } else {
            intake.setIntakePosition(intake.intakeSafePosition);
            claw.openBothClaw();
            sleep(300);
            claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
            sleep(400);
            delivery.resetSlide();
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
            sleep(200);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            sleep(200);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        }

        intake.resetMotor();
        delivery.resetMotor();

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
                    double leftDistance = intake.getLeftPixelSensor();
                    double rightDistance = intake.getRightPixelSensor();
                    if (leftDistance < intake.leftPixelDetectDistance) claw.closeLeftClaw();
                    if (rightDistance < intake.rightPixelDetectDistance) claw.closeRightClaw();
                    //if (claw.getLeftClawSensor() && claw.getRightClawSensor()) {
                    //    robotState = RobotState.CLAW_CLOSE;
                    //}

                    if ((leftDistance < intake.leftPixelDetectDistance) &&
                            (rightDistance < intake.rightPixelDetectDistance)) {
                        robotState = RobotState.CLAW_CLOSE;
                        claw.closeBothClaw();
                    }
                    clawCloseTimer.reset();
                    intakeBackSpinTimer.reset();
                    break;
                case CLAW_CLOSE:
                    if (clawCloseTimer.milliseconds() > claw.clawCloseTime) {
                        robotState = RobotState.INTAKE_BACKSPIN;
                        intake.intakeBackSpin();
                    }
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
                        if (controllers.deliveryKey == 'a') {
                            v4Bar.setV4BarPosition(controllers.target[0][1]);
                        }

                        if (controllers.deliveryKey == 'x') {
                           v4Bar.setV4BarPosition(controllers.target[1][1]);
                        }

                        if (controllers.deliveryKey == 'y') {
                           v4Bar.setV4BarPosition(controllers.target[2][1]);
                        }

                        if (controllers.deliveryKey == '1') {
                            v4Bar.setV4BarPosition(controllers.target[3][1]);
                        }
                        robotState = RobotState.CLAW_ANGLE_STAGE2;
                        v4BarUpTimer.reset();
                    }
                    break;
                case CLAW_ANGLE_STAGE2:
                    if (v4BarUpTimer.milliseconds() > v4Bar.v4BarUpStage2Time) {
                        robotState = RobotState.DELIVERY_READY;
                    }
                    break;
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
                    if (controllers.deliveryKey == '1') {
                        delivery.slideRunToPosition_Encoder((int) controllers.target[3][0], delivery.slideRunHighVelocity);
                        v4Bar.setV4BarPosition(controllers.target[3][1]);
                        claw.setClawAnglePosition(controllers.target[3][2]);
                        delivery.slideAngleRunToPosition((int)controllers.target[3][3]);
                    }
                    clawOpenTimer.reset();
                    break;
                case CLAW_OPEN:
                    if (clawOpenTimer.milliseconds() > claw.clawOpenTime) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
                        robotState=RobotState.V4BAR_UP;
                        waitingTimer.reset();
                    }
                    break;
                case V4BAR_UP:
                    if (waitingTimer.milliseconds() > 150) {
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        robotState = RobotState.SLIDE_DOWN;
                    }
                    break;
                case SLIDE_DOWN:
                    if (((Math.abs(delivery.getMotor1Position()) - 250) < 0) ||
                            (Math.abs(delivery.getMotor2Position()) - 250 < 0))
                    {
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
                        delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
                        robotState = RobotState.SLIDE_ANGLE_DOWN;
                        v4BarDownTimer.reset();
                    }
                    break;
                case SLIDE_ANGLE_DOWN: //slide angle has to be down first
                    if (((Math.abs(delivery.getSlideAnglePosition()) - 10) < 0) &&
                            (v4BarDownTimer.milliseconds() > v4Bar.v4BarDownTime) &&
                            (((Math.abs(delivery.getMotor1Position()) - 10) < 0) ||
                            (Math.abs(delivery.getMotor2Position()) - 10 < 0)))
                    {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
                        robotState = RobotState.V4BAR_DOWN_MIDDLE;
                        waitingTimer.reset();
                    }
                    break;
                case V4BAR_DOWN_MIDDLE:
                    if (waitingTimer.milliseconds() > 250) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
                        claw.setClawAnglePosition(claw.clawAngleIntake);
                        robotState = RobotState.DELIVERY_DONE;
                        waitingTimer.reset();
                    }
                    break;
                case DELIVERY_DONE:
                    if (waitingTimer.milliseconds() > 200) {
                        intake.setIntakePosition(intake.intakeCenterPosition);
                        robotState = RobotState.IDLE;
                        delivery.resetMotor();
                        controllers.deliveryKey = '\0';
                    }
                    break;
            }

            // ----------------------------------------- //
            // ---------- DRONE HANGER FSM START ----------- //
            // ----------------------------------------- //

            switch (robotState) {

                case DRONE_HANGER_START:
                    robotState = RobotState.DH_V4BAR_UP_STAGE1;
                    waitingTimer.reset(); //waiting intake bar go to safe position
                    break;
                case DH_V4BAR_UP_STAGE1:
                    if (waitingTimer.milliseconds() > 200) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
                        robotState = RobotState.DH_CLAW_ANGLE_STAGE1;
                        clawAngleTimer.reset();
                    }
                    break;
                case DH_CLAW_ANGLE_STAGE1:
                    if (clawAngleTimer.milliseconds() > claw.clawAngleStage1Time) {
                        claw.setClawAnglePosition(claw.clawAngleDeliveryStage1);
                        robotState = RobotState.DH_V4BAR_UP_STAGE2;
                        waitingTimer.reset();
                    }
                    break;
                case DH_V4BAR_UP_STAGE2:
                    if (waitingTimer.milliseconds() > 200) {
                        v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
                        robotState = RobotState.DH_CLAW_ANGLE_STAGE2;
                        v4BarUpTimer.reset();
                    }
                    break;
                case DH_CLAW_ANGLE_STAGE2:
                    if (waitingTimer.milliseconds() > v4Bar.v4BarUpStage2Time) {
                        delivery.slideRunToPosition_Encoder(delivery.slideExtendDrone, delivery.slideRunHighVelocity);
                        delivery.slideAngleRunToPosition(delivery.slideAngleDrone);
                        robotState = RobotState.DH_SLIDE_UP;
                    }
                    break;
                case DH_SLIDE_UP:
                    if (controllers.droneLaunched) {
                        droneLaunchTimer.reset();
                        robotState = RobotState.HANGING_START;
                    }
                    break;
                case HANGING_START:
                    if (droneLaunchTimer.milliseconds() > 500) {
                        intake.setIntakePosition(intake.intakeHangerPosition);
                        v4Bar.setV4BarPosition(v4Bar.v4BarHangerReadyPosition);
                        delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideRunHighVelocity);
                        delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
                        claw.setClawAnglePosition(claw.clawAngleHanger);
                        robotState = RobotState.HANGING_READY;
                    }
                    break;
                case HANGING_READY:
                    if (controllers.hanging) {
                        robotState = RobotState.IDLE;
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
//        telemetry.addData("motor1 position", delivery.getMotor1Position());
//        telemetry.addData("motor2 position", delivery.getMotor2Position());
//        telemetry.addData("slide angle position", delivery.getSlideAnglePosition());
//        telemetry.addData("v4Bar left position", v4Bar.getV4BarLeftPosition());
//        telemetry.addData("v4Bar right position", v4Bar.getV4BarRightPosition());
//        telemetry.addData("claw angle position", claw.getClawAngle());
//        telemetry.addData("intake position", intake.getIntakeDownPosition());
//        telemetry.addData("motor1 current", intake.getMotor1Current());
//        telemetry.addData("motor2 current", intake.getMotor2Current());
//        telemetry.addData("left pixel on", intake.getLeftPixelSensor());
//        telemetry.addData("right pixel on", intake.getRightPixelSensor());
//        telemetry.addData("left claw on", claw.getLeftClawSensor());
//        telemetry.addData("right claw on", claw.getRightClawSensor());
//        telemetry.addData("slide angle motor current", delivery.getSlideAngleMotorCurrent());
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
