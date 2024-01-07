package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "DifferentialPIDTest2", group = "Test")
public class DifferentialPIDTest2 extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private PIDController controller1, controller2;

    public static double p1 = 0.015, i1 = 0, d1 = 0.0003;
    public static double f1 = 0.032;

    public static double p2 = 0.015, i2 = 0, d2 = 0.0003;
    public static double f2 = 0.032;

    public static int target1 = 300;
    public static int target2 = 150;
    public static int target3 = 0;

    boolean intake = false;
    double power1 = 0, power2 = 0;
    int position1 = 0, position2 = 0;
    int target = 0;
    int positionTolerance = 3;

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    @Override

    public void runOpMode() throws InterruptedException{

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        //motor1.setDirection(DcMotorEx.Direction.REVERSE);
        //motor2.setDirection(DcMotorEx.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2, i2, d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (!intake) {

                controller1.setPID(p1, i1, d1);
                controller2.setPID(p2, i2, d2);

                position1 = motor1.getCurrentPosition();
                position2 = motor2.getCurrentPosition();

                if (gamepad1.a) target = target1;
                if (gamepad1.b) target = target2;
                if (gamepad1.y) target = target3;

                motor1.setTargetPositionTolerance(positionTolerance);
                motor2.setTargetPositionTolerance(positionTolerance);

                double pid1 = controller1.calculate(position1, target);
                double pid2 = controller2.calculate(position2, target);
                power1 = pid1 + f1;
                power2 = pid2 + f2;

                motor1.setPower(power1);
                motor2.setPower(power2);

            }

            if (gamepad1.x) {
                motor1.setPower(0);
                motor2.setPower(0);
                intake = false;

                //prepare for delivery mode
                target = 0;
                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            if (gamepad1.start) {
                motor1.setTargetPositionTolerance(positionTolerance);
                motor2.setTargetPositionTolerance(positionTolerance);
                motor1.setTargetPosition(400);
                motor2.setTargetPosition(400);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setPower(1);
                motor2.setPower(1);
            }

            if (gamepad1.back) {
                motor1.setTargetPositionTolerance(positionTolerance);
                motor2.setTargetPositionTolerance(positionTolerance);
                motor1.setTargetPosition(0);
                motor2.setTargetPosition(0);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setPower(1);
                motor2.setPower(1);
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
