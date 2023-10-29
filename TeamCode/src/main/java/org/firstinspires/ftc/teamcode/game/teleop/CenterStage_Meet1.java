package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "TeleOp")
public class CenterStage_Meet1 extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;

    ElapsedTime armTimer;

    public double driveSpeedRatio = 1.0;
    public ArmState armState = ArmState.IDLE;
    

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Claw claw = new Claw();
        claw.init(hardwareMap);

        claw.wristDown();
        claw.openArm();

        Controllers controllers = new Controllers(this, claw);

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

            controllers.updateCopies(gamepad1, gamepad2);

            drive.update();

            controllers.readInputs(gamepad1, gamepad2);

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
