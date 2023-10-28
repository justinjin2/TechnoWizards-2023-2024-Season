package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "TeleOp")
public class TeleOp extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;

    ElapsedTime armTimer;

    private double driveSpeedRatio = 1.0;
    public ArmState armState = ArmState.IDLE;
    

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Claw claw = new Claw();
        claw.init(hardwareMap);

        claw.wristDown();
        claw.openArm();

        armState = ArmState.IDLE;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        displayPoseTelemetry();

        loopTimer = new ElapsedTime();
        armTimer = new ElapsedTime();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * driveSpeedRatio,
                            -gamepad1.left_stick_x * driveSpeedRatio,
                            -gamepad1.right_stick_x * driveSpeedRatio
                    )
            );

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            drive.update();


            if (gamepad1.right_trigger > 0) {
                driveSpeedRatio = 0.35;
            } else {
                driveSpeedRatio = 1.0;
            }

            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                claw.wristDown();
                claw.closeArm();
                armTimer.reset();
                armState = ArmState.PIXEL_GRAB;
            }
            if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                claw.openArm();
                armTimer.reset();
                armState = ArmState.CLAW_OPEN;
            }
            if (!currentGamepad1.a && previousGamepad1.a) {
                claw.clawSlideRunToPosition(claw.slideLow);
            }
            if (!currentGamepad1.x && previousGamepad1.x) {
                claw.clawSlideRunToPosition(claw.slideMedium);
            }
            if (!currentGamepad1.y && previousGamepad1.y) {
                claw.clawSlideRunToPosition(claw.slideMaxHeight);
            }
            if (!currentGamepad1.b && previousGamepad1.b) {
                claw.clawSlideRunToPosition(claw.slideStart);
                claw.wristDown();
                claw.openArm();
            }
            if (gamepad1.dpad_up) {
                claw.slideManualRun(claw.smallSlideManual);
            }
            if (gamepad1.dpad_down) {
                claw.slideManualRun(-claw.smallSlideManual);
            }


/**
 * --- OLD TESTING CODE ---
 *  if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {claw.slideManualRun(claw.slideManual);}
 *  if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {claw.slideManualRun(-claw.slideManual);}
 *  if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {claw.openArm();}
 *
 *  if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {claw.wristUp();}
 *  if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {claw.wristDown();}
 *
 *  if (!currentGamepad1.y && previousGamepad1.y) {claw.slideManualRun(claw.slideManual);}
 *  if (!currentGamepad1.a && previousGamepad1.a) {claw.slideManualRun(-claw.slideManual);}
 */

            switch (armState) {
                case PIXEL_GRAB:
                    if ((armTimer.milliseconds() > claw.armCloseTime)) {
                        claw.wristUp();
                        armTimer.reset();
                        armState = ArmState.WRIST_READY;
                    }
                    break;
                case WRIST_READY:
                    if ((armTimer.milliseconds() > claw.armWristUpTime)) {
                        armState = ArmState.IDLE;
                    }
                    break;
                case CLAW_OPEN:
                    if ((armTimer.milliseconds() > claw.armOpenTime)) {
                        claw.clawSlideRunToPosition(claw.slideStart);
                        armTimer.reset();
                        armState = ArmState.SLIDE_DOWN;
                    }
                    break;
                case SLIDE_DOWN:
                    if ((armTimer.milliseconds() > claw.slideHalfDownTime)) {
                        claw.wristDown();
                        armState = ArmState.IDLE;
                    }
                    break;
            }

            /**
             * -- OLD TESTING CODE --
             *             if (gamepad2.dpad_up) intake.intakeSlideRunToPosition(intakeSlideRunManual);
             *             if (gamepad2.dpad_down) intake.intakeSlideRunToPosition(0);
             *
             *             if (gamepad1.dpad_up) robot.deliverySlideManualRun(deliverySlideRunManual);
             *             if (gamepad1.dpad_down)robot.deliverySlideManualRun(-deliverySlideRunManual);
             *
             *             if (gamepad1.right_bumper) {
             *                 robot.deliveryClawClose();
             *                 deliveryServoTimer.reset();
             *                 deliveryState = Delivery_State.DELIVERY_CONE_GRAB;
             *             }
             *
             *             if (gamepad1.left_bumper) robot.deliveryClawOpen();
             *             if (gamepad1.b) robot.deliverySlideRunToPosition(robot.deliverySlide_Start);
             *
             *             if (gamepad1.y) {
             *                 // Y is pressed, intake side start extending
             *                 robot.deliverySlideRunToPosition(robot.deliverySlide_High);
             *                 deliveryState = Delivery_State.DELIVERY_EXTEND;
             *                 deliverySlideTimer.reset();
             *             }
             *             if (gamepad1.x) {
             *                 // X is pressed, intake side start extending
             *                 robot.deliverySlideRunToPosition(robot.deliverySlide_Middle);
             *                 deliveryState = Delivery_State.DELIVERY_EXTEND;
             *                 deliverySlideTimer.reset();
             *             }
             *             if (gamepad1.a) {
             *                 // Y is pressed, intake side start extending
             *                 robot.deliverySlideRunToPosition(robot.deliverySlide_Low);
             *                 deliveryState = Delivery_State.DELIVERY_EXTEND;
             *                 deliverySlideTimer.reset();
             *             }
             */

            displayTelemetry(drive, claw);

        }
    }

    public void displayPoseTelemetry() {
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());
    }

    public void displayTelemetry(SampleMecanumDrive drive, Claw claw) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Loop Timer", loopTimer.milliseconds());
        telemetry.addData("Left Slide Position", claw.clawMotorLeftPosition());
        telemetry.addData("Right Slide Position", claw.clawMotorRightPosition());
        telemetry.update();
    }

}
