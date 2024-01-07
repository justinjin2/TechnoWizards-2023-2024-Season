package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw_Meet1;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "TeleOp")
public class CenterStage_Meet1 extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;
    ElapsedTime armTimer;

    private double driveSpeedRatio = 1.0;
    private ArmState armState = ArmState.IDLE;
    
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        Claw_Meet1 clawOld = new Claw_Meet1();
        clawOld.init(hardwareMap);

        clawOld.wristDown();
        clawOld.openArm();
        clawOld.droneClose();

        Controllers controllers = new Controllers(this, clawOld);

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
                    if ((armTimer.milliseconds() > clawOld.armCloseTime)) {
                        clawOld.wristUp();
                        armTimer.reset();
                        armState = ArmState.WRIST_READY;
                    }
                    break;
                case WRIST_READY:
                    if ((armTimer.milliseconds() > clawOld.armWristUpTime)) {
                        armState = ArmState.IDLE;
                    }
                    break;
                case CLAW_OPEN:
                    if ((armTimer.milliseconds() > clawOld.armOpenTime)) {
                        clawOld.clawSlideRunToPosition(clawOld.slideStart);
                        armTimer.reset();
                        armState = ArmState.SLIDE_DOWN;
                    }
                    break;
                case SLIDE_DOWN:
                    if ((armTimer.milliseconds() > clawOld.slideHalfDownTime)) {
                        clawOld.wristDown();
                        armState = ArmState.IDLE;
                    }
                    break;
            }

            displayTelemetry(drive, clawOld);
        }
    }

    public void displayPoseTelemetry() {
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());
    }

    public void displayTelemetry(SampleMecanumDrive drive, Claw_Meet1 clawOld) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Loop Timer", loopTimer.milliseconds());
        telemetry.addData("Left Slide Position", clawOld.clawMotorLeftPosition());
        telemetry.addData("Right Slide Position", clawOld.clawMotorRightPosition());
        telemetry.update();
    }

    public void setDriveSpeedRatio(double ratio) {
        driveSpeedRatio = ratio;
    }

    public ArmState getArmState() {
        return armState;
    }

    public void setArmState(ArmState state) {
        armState = state;
    }

}
