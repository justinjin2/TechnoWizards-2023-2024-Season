package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Slide;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "TeleOp")
public class CenterStage_Meet2 extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;

    private double driveSpeedRatio = 1.0;
    

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        Slide slide = new Slide();
        slide.init(hardwareMap);

        Controllers controllers = new Controllers(this, slide);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        //displayPoseTelemetry();

        loopTimer = new ElapsedTime();
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
            telemetry.addData("Motor 1 Position:", slide.motor1Position());
            telemetry.addData("Motor 2 Position:", slide.motor2Position());
            telemetry.update();
        }
    }

    public void displayPoseTelemetry() {
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());

    }

    public void displayTelemetry(SampleMecanumDrive drive) {
        telemetry.update();
    }

    public void setDriveSpeedRatio(double ratio) {
        driveSpeedRatio = ratio;
    }


}
