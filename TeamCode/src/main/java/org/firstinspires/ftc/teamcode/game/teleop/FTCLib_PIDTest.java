package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

@Config
@TeleOp(name = "FTCLib_PIDTest", group = "Test")
public class FTCLib_PIDTest extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private PIDController controller1, controller2;

    public static double p1 = 0.015, i1 = 0, d1 = 0.0003;
    public static double f1 = 0.032;

    public static double p2 = 0.015, i2 = 0, d2 = 0.0003;
    public static double f2 = 0.032;

    public static double KS = 0, KV = 0, KA = 0;

    public static int target1 = 150;
    public static int target2 = 350;
    public static int target3 = 550;

    boolean intake = false;
    double power1 = 0, power2 = 0;
    int position1 = 0, position2 = 0;
    int target = 0;
    int positionTolerance = 3;

    private Motor motor1;
    private Motor motor2;

    @Override
    public void runOpMode() throws InterruptedException{

        Motor motor1 = new Motor(hardwareMap, "motor1", Motor.GoBILDA.RPM_1150);
        Motor motor2 = new Motor(hardwareMap, "motor2", Motor.GoBILDA.RPM_1150);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

        //motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        //motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        //motor1.setInverted(true);
        motor2.setInverted(true);

        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor1.setRunMode(Motor.RunMode.RawPower);

        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setRunMode(Motor.RunMode.RawPower);

        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2, i2, d2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1.resetEncoder();
        motor2.resetEncoder();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (!intake) {

                controller1.setPID(p1, i1, d1);
                controller2.setPID(p2, i2, d2);

                if (gamepad1.a) target = target1;
                if (gamepad1.x) target = target2;
                if (gamepad1.y) target = target3;

                motor1.setTargetPosition(target);
                motor2.setTargetPosition(target);

                position1 = motor1.getCurrentPosition();
                position2 = motor2.getCurrentPosition();

                double pid1 = controller1.calculate(position1, target);
                double pid2 = controller2.calculate(position2, target);

                power1 = pid1 + f1;
                power2 = pid2 + f2;

                if (!motor1.atTargetPosition()) motor1.set(power1);
                if (!motor2.atTargetPosition()) motor2.set(power2);

            }

            if (gamepad1.b) {
                motor1.set(0);
                motor2.set(0);

                motor1.resetEncoder();
                motor2.resetEncoder();
                target = 0; //loop back to delivery if not intake

                intake = false;
            }

            if (gamepad1.right_bumper) {
                motor1.set(-0.5);
                motor2.set(0.5);
                intake = true;
            }

            if (gamepad1.left_bumper) {
                motor1.set(0.5);
                motor2.set(-0.5);
                intake = true;
            }

            telemetry.addData("power1 = ", power1);
            telemetry.addData("power2 = ", power2);
            telemetry.addData("position1 = ", motor1.getCurrentPosition());
            telemetry.addData("position2 = ", motor2.getCurrentPosition());
            telemetry.addData("target = ", target);
            telemetry.update();
        }
    }

}
