package org.firstinspires.ftc.teamcode.game.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.game.autonomous.Auto_Region;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "TeleOp")

public class CenterStage_Test extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime loopTimer;

    private double driveSpeedRatio = 1.0;
    private RobotState robotState = RobotState.IDLE;
    private AprilTagDetector aprilTagDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        Intake intake = new Intake();
        Delivery delivery = new Delivery();
        V4Bar v4Bar = new V4Bar();
        Claw claw = new Claw();

        aprilTagDetector = new AprilTagDetector(hardwareMap);
        aprilTagDetector.init();

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        Controllers_Test controllers = new Controllers_Test(this, intake, delivery, v4Bar, claw);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        intake.setIntakePosition(intake.intakeCenterPosition);
        //delivery.resetSlide();
//        claw.setClawAnglePosition(0.48);
//        v4Bar.setV4BarPosition(Auto_Region.V4BAR_DELIVERY);
        //intake.resetMotor();
        //delivery.resetMotor(); //reset all motors encoder
        //intake.setIntakePosition(intake.intakeCenterPosition);
        //sleep(400);
        //claw.setClawAnglePosition(claw.clawAngleIntake);
        //v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        //intake.setIntakePosition(intake.intakeSafePosition);
        //claw.openBothClaw();

        displayPoseTelemetry();

        loopTimer = new ElapsedTime();
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        if (isStopRequested()) return;

        delivery.redLED.setMode(DigitalChannel.Mode.OUTPUT);
        delivery.greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            // Will run one bulk read per cycle,
            // because the caches are being handled manually and cleared
            // once a loop
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

            switch (robotState) {
                case PIXEL_GRAB:

                    break;
                case WRIST_READY:

                    break;
                case CLAW_OPEN:

                    break;
                case SLIDE_DOWN:

                    break;
            }

            if (claw.getLeftClawSensor()) {
                delivery.greenLED.setState(true);
                delivery.redLED.setState(false);
            }

            if (claw.getRightClawSensor()) {
                delivery.greenLED.setState(false);
                delivery.redLED.setState(true);
            }

            if (claw.getLeftClawSensor() && claw.getRightClawSensor()) {
                delivery.redLED.setState(false);
                delivery.greenLED.setState(false);
            }

            if (!claw.getLeftClawSensor() && !claw.getRightClawSensor()) {
                delivery.redLED.setState(true);
                delivery.greenLED.setState(true);
            }

            displayTelemetry(drive, intake, delivery, v4Bar, claw);

        }
    }

    public void displayPoseTelemetry() {
        telemetry.addData("startX", PoseStorage.currentPose.getX());
        telemetry.addData("startY", PoseStorage.currentPose.getY());
        telemetry.addData("startHeading", PoseStorage.currentPose.getHeading());
    }

    public void displayTelemetry(SampleMecanumDrive drive, Intake intake, Delivery delivery,
                                 V4Bar v4Bar, Claw claw) {
        //Pose2d poseEstimate = drive.getPoseEstimate();
        //telemetry.addData("x", poseEstimate.getX());
        //telemetry.addData("y", poseEstimate.getY());
        //telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("back left ultrasound", intake.getUltrasonicBackLeft());
        telemetry.addData("back right ultrasound", intake.getUltrasonicBackRight());
        telemetry.addData("side left ultrasound", delivery.getUltrasonicLeft());
        telemetry.addData("side right ultrasound", delivery.getUltrasonicRight());
        telemetry.addData("left slide sensor", delivery.getLeftSlideSensor());
        telemetry.addData("right slide sensor", delivery.getRightSlideSensor());
        telemetry.addData("slide angle sensor", delivery.getSlideAngleSensor());
        telemetry.addData("motor1 position", delivery.getMotor1Position());
        telemetry.addData("motor2 position", delivery.getMotor2Position());
        telemetry.addData("slide angle position", delivery.getSlideAnglePosition());
        telemetry.addData("v4Bar left position", v4Bar.getV4BarLeftPosition());
        telemetry.addData("v4Bar right position", v4Bar.getV4BarRightPosition());
        telemetry.addData("claw angle position", claw.getClawAngle());
        telemetry.addData("intake position", intake.getIntakeDownPosition());
        telemetry.addData("motor1 current", intake.getMotor1Current());
        telemetry.addData("motor2 current", intake.getMotor2Current());
        telemetry.addData("test servo position", claw.getTestServoPosition());
        telemetry.addData("left pixel on", intake.getLeftPixelSensor());
        telemetry.addData("right pixel on", intake.getRightPixelSensor());
        telemetry.addData("left claw on", claw.getLeftClawSensor());
        telemetry.addData("right claw on", claw.getRightClawSensor());
        telemetry.addData("Loop Timer", loopTimer.milliseconds());
        telemetry.addData("Tag Bearings: ", aprilTagDetector.getRawBearings());
        telemetry.addData("Odometry Heading", Math.round(drive.getPoseEstimate().getHeading()));
        telemetry.update();
    }

    public void setDriveSpeedRatio(double ratio) {
        driveSpeedRatio = ratio;
    }

}
