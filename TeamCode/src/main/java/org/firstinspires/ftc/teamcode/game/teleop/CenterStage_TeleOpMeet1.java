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
    ElapsedTime deliveryServoTimer;

//    public enum Delivery_State {
//        DELIVERY_IDLE,
//        DELIVERY_CONE_GRAB,
//        DELIVERY_READY,
//        DELIVERY_EXTEND,
//        DELIVERY_RELEASE
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = dashboard.getTelemetry();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Claw claw = new Claw();
        claw.init(hardwareMap);

        claw.resetArm();
        claw.resetWrist();

//        Delivery_State deliveryState = Delivery_State.DELIVERY_IDLE;

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
        deliveryServoTimer = new ElapsedTime();
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
            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {claw.closeArm();}
            if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {claw.openArm();}

            if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {claw.wristUp();}
            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {claw.wristDown();}

            if (!currentGamepad1.y && previousGamepad1.y) {claw.slideManualRun(claw.slideManual);}
            if (!currentGamepad1.a && previousGamepad1.a) {claw.slideManualRun(-claw.slideManual);}

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
