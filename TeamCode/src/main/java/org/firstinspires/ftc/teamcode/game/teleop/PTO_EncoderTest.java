package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;

import java.util.List;

@Config
@TeleOp(name = "PTO_EncoderTest", group = "Test")
public class PTO_EncoderTest extends LinearOpMode {

    ElapsedTime loopTimer;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int target1 = 150;
    public static int target2 = 350;
    public static int target3 = 550;

    boolean intake = false;
    double power1 = 0, power2 = 0;
    int position1 = 0, position2 = 0;
    int target = 0;
    int positionTolerance = 3;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException{

        Delivery delivery = new Delivery();
        PTO pto = new PTO();
        Intake intake = new Intake();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loopTimer = new ElapsedTime();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        delivery.init(hardwareMap);
        pto.init(hardwareMap);
        intake.init(hardwareMap);

        delivery.resetMotor();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Will run one bulk read per cycle,
            // even as frontLeftMotor.getCurrentPosition() is called twice
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.a) delivery.slideRunToPosition_Encoder(target1, delivery.slideRunHighVelocity);
            if (gamepad1.x) delivery.slideRunToPosition_Encoder(target2, delivery.slideRunHighVelocity);
            if (gamepad1.y) delivery.slideRunToPosition_Encoder(target3, delivery.slideRunHighVelocity);
            if (gamepad1.b) delivery.slideRunToPosition_Encoder(0, delivery.slideRunLowVelocity);

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) delivery.slideRunToPositionManual_Encoder(50);
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) delivery.slideRunToPositionManual_Encoder(-50);

            if (gamepad1.dpad_left) {
                pto.motor1.setPower(0);
                pto.motor2.setPower(0);

                pto.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pto.motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                pto.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pto.motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.right_bumper) {
                pto.motor1.setPower(-0.8);
                pto.motor2.setPower(0.8);
            }

            if (gamepad1.left_bumper) {
                pto.motor1.setPower(0.8);
                pto.motor2.setPower(-0.8);
            }

            telemetry.addData("power1 = ", pto.motor1.getVelocity());
            telemetry.addData("power2 = ", pto.motor2.getVelocity());
            telemetry.addData("position1 = ", delivery.getMotor1Position());
            telemetry.addData("position2 = ", delivery.getMotor2Position());
            telemetry.addData("motor1 current", intake.getMotor1Current());
            telemetry.addData("motor2 current", intake.getMotor2Current());
            telemetry.addData("loopTimer", loopTimer.milliseconds());
            telemetry.update();
        }
    }

}
