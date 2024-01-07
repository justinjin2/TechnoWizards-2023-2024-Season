package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.PTO;

import java.util.List;

@Config
@TeleOp(name = "RoadRunner_feedForwardTest", group = "Test")
public class RoadRunner_feedForwardTest extends LinearOpMode {

    ElapsedTime loopTimer;

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p1 = 0.005, i1 = 0.002, d1 = 0.0004;
    public static double f1 = 0;

    public static double p2 = 0.005, i2 = 0.002, d2 = 0.0004;
    public static double f2 = 0;

    public double KV = 0, KA = 0, KStatic = 0;

    private PIDCoefficients coffes1 = new PIDCoefficients(p1, i1, d1);
    private PIDCoefficients coffes2 = new PIDCoefficients(p2, i2, d2);

    private PIDFController controller1 = new PIDFController(coffes1, KV, KA, KStatic);
    private PIDFController controller2 = new PIDFController(coffes2, KV, KA, KStatic);

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loopTimer = new ElapsedTime();

        delivery.init(hardwareMap);
        pto.init(hardwareMap);

        delivery.resetMotor();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Will run one bulk read per cycle,
            // even as Motor.getCurrentPosition() is called twice
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (!intake) {

                if (gamepad1.a) target = target1;
                if (gamepad1.x) target = target2;
                if (gamepad1.y) target = target3;
                if (gamepad1.b) target = 0;


                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up
                    && target <= 500) target += 50;
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down
                    && target >= 50) target -= 50;

                controller1.setTargetPosition(target);
                controller2.setTargetPosition(target);

                position1 = delivery.getMotor1Position();
                position2 = delivery.getMotor2Position();

                double pid1 = controller1.update(position1);
                double pid2 = controller2.update(position2);

                power1 = pid1 + f1;
                power2 = pid2 + f2;

                pto.motor1.setPower(power1);
                pto.motor2.setPower(power2);
            }

            if (gamepad1.dpad_left) {
                motor1.setPower(0);
                motor2.setPower(0);

                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                target = 0; //loop back to delivery if not intake

                intake = false;
            }

            if (gamepad1.right_bumper) {
                motor1.setPower(-0.8);
                motor2.setPower(0.8);
                intake = true;
            }

            if (gamepad1.left_bumper) {
                motor1.setPower(0.8);
                motor2.setPower(-0.8);
                intake = true;
            }

            telemetry.addData("power1 = ", power1);
            telemetry.addData("power2 = ", power2);
            telemetry.addData("position1 = ", delivery.getMotor1Position());
            telemetry.addData("position2 = ", delivery.getMotor2Position());
            telemetry.addData("target = ", target);
            telemetry.addData("loopTimer", loopTimer.milliseconds());
            telemetry.update();
        }
    }

}
