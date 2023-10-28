package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw;

import dalvik.system.DelegateLastClassLoader;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@TeleOp(group = "advanced")
public class CenterStage_TeleOpMeet1 extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;
    ElapsedTime deliverySlideTimer;
    ElapsedTime armCloseTimer;
    ElapsedTime armOpenTimer;
    ElapsedTime wristUpTimer;
    ElapsedTime slideHalfDownTimer;

    public enum Arm_State {
        ARM_IDLE,
        ARM_PIXEL_GRAB,
        ARM_WRIST_READY,
        ARM_CLAW_OPEN,
        ARM_SLIDE_DOWN,
        ARM_EXTEND,
        ARM_RELEASE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Claw claw = new Claw();
        claw.init(hardwareMap);

        claw.wristDown();
        claw.openArm();


        Arm_State armState = Arm_State.ARM_IDLE;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int deliverySlideRunManual = 120;
        int intakeSlideRunManual = 300;
        double manualDrivePower = 0.8;

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());

        loopTimer = new ElapsedTime();
        deliverySlideTimer = new ElapsedTime();
        armCloseTimer = new ElapsedTime();
        armOpenTimer = new ElapsedTime();
        wristUpTimer = new ElapsedTime();
        slideHalfDownTimer = new ElapsedTime();
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x ,
                            -gamepad1.right_stick_x //;;  ;;
                    )
            );
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Update everything. Odometry. Etc.
            drive.update();

            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                claw.wristDown();
                claw.closeArm();
                armCloseTimer.reset();
                armState = Arm_State.ARM_PIXEL_GRAB;
            }
            if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                claw.openArm();
                armOpenTimer.reset();
                armState = Arm_State.ARM_CLAW_OPEN;
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
            if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {claw.slideManualRun(claw.slideManual);}
            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {claw.slideManualRun(-claw.slideManual);}

            //old testing code
//            if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {claw.openArm();}
//
//            if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {claw.wristUp();}
//            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {claw.wristDown();}
//
//            if (!currentGamepad1.y && previousGamepad1.y) {claw.slideManualRun(claw.slideManual);}
//            if (!currentGamepad1.a && previousGamepad1.a) {claw.slideManualRun(-claw.slideManual);}

            switch (armState) {
                case ARM_PIXEL_GRAB:
                    if ((armCloseTimer.milliseconds() > claw.armCloseTime)) {
                        claw.wristUp();
                        wristUpTimer.reset();
                        armState = Arm_State.ARM_WRIST_READY;
                    }
                    break;
                case ARM_WRIST_READY:
                    if ((wristUpTimer.milliseconds() > claw.armWristUpTime)) {
                        armState = Arm_State.ARM_IDLE;
                    }
                    break;
                case ARM_CLAW_OPEN:
                    if ((armOpenTimer.milliseconds() > claw.armOpenTime)) {
                        claw.clawSlideRunToPosition(claw.slideStart);
                        slideHalfDownTimer.reset();
                        armState = Arm_State.ARM_SLIDE_DOWN;
                    }
                    break;
                case ARM_SLIDE_DOWN:
                    if ((slideHalfDownTimer.milliseconds() > claw.slideHalfDownTime)) {
                        claw.wristDown();
                        armState = Arm_State.ARM_IDLE;
                    }
                    break;
            }

//            if (gamepad2.dpad_up) intake.intakeSlideRunToPosition(intakeSlideRunManual);
//            if (gamepad2.dpad_down) intake.intakeSlideRunToPosition(0);
//
//            if (gamepad1.dpad_up) robot.deliverySlideManualRun(deliverySlideRunManual);
//            if (gamepad1.dpad_down)robot.deliverySlideManualRun(-deliverySlideRunManual);
//
//            if (gamepad1.right_bumper) {
//                robot.deliveryClawClose();
//                deliveryServoTimer.reset();
//                deliveryState = Delivery_State.DELIVERY_CONE_GRAB;
//            }
//
//            if (gamepad1.left_bumper) robot.deliveryClawOpen();
//            if (gamepad1.b) robot.deliverySlideRunToPosition(robot.deliverySlide_Start);
//
//            if (gamepad1.y) {
//                // Y is pressed, intake side start extending
//                robot.deliverySlideRunToPosition(robot.deliverySlide_High);
//                deliveryState = Delivery_State.DELIVERY_EXTEND;
//                deliverySlideTimer.reset();
//            }
//            if (gamepad1.x) {
//                // X is pressed, intake side start extending
//                robot.deliverySlideRunToPosition(robot.deliverySlide_Middle);
//                deliveryState = Delivery_State.DELIVERY_EXTEND;
//                deliverySlideTimer.reset();
//            }
//            if (gamepad1.a) {
//                // Y is pressed, intake side start extending
//                robot.deliverySlideRunToPosition(robot.deliverySlide_Low);
//                deliveryState = Delivery_State.DELIVERY_EXTEND;
//                deliverySlideTimer.reset();
//            }

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Loop Timer", loopTimer.milliseconds());
            telemetry.addData("Left Slide Position", claw.clawMotorLeftPosition());
            telemetry.addData("Right Slide Position", claw.clawMotorRightPosition());
            telemetry.update();
        }
    }
}
