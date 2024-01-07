package org.firstinspires.ftc.teamcode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.sql.Time;

@Disabled
@Config
@TeleOp(name = "RoadRunner_MotionProfileTest", group = "Test")
public class RoadRunner_MotionProfileTest extends LinearOpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double kp1 = 0.0135, ki1 = 0.0002, kd1 = 0.0003;
    public static double kf1 = 0;

    public static double kp2 = 0.0135, ki2 = 0.0002, kd2 = 0.0003;
    public static double kf2 = 0;

    //public static double KS = 0, KV = 0, KA = 0;

    private PIDCoefficients coffes1 = new PIDCoefficients(kp1, ki1, kd1);
    private PIDCoefficients coffes2 = new PIDCoefficients(kp2, ki2, kd2);

    private PIDFController controller1 = new PIDFController(coffes1);
    private PIDFController controller2 = new PIDFController(coffes2);

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(60, 0, 0),
            25,
            40,
            100
    );

    MotionState state;

    double Max_Acceleration, Max_Velocity, Distance, Elapsed_time;

    public static int target1 = 150;
    public static int target2 = 350;
    public static int target3 = 550;

    boolean intake = false;
    double power1 = 0, power2 = 0;
    int position1 = 0, position2 = 0;
    int target = 0;
    int positionTolerance = 3;

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (!intake) {

                if (gamepad1.a) target = target1;
                if (gamepad1.b) target = target2;
                if (gamepad1.y) target = target3;

                controller1.setTargetPosition(target);
                controller2.setTargetPosition(target);

                position1 = motor1.getCurrentPosition();
                position2 = motor2.getCurrentPosition();

                trapezoidal_profile(Max_Acceleration, Max_Velocity, Distance, Elapsed_time);

                double pid1 = controller1.update(position1);
                double pid2 = controller2.update(position2);

                telemetry.addData("pid1", pid1);
                telemetry.addData("pid2", pid2);
                telemetry.update();

                power1 = pid1 + kf1;
                power2 = pid2 + kf2;

                motor1.setPower(power1);
                motor2.setPower(power2);
            }

            if (gamepad1.x) {
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
                motor1.setPower(-0.5);
                motor2.setPower(0.5);
                intake = true;
            }

            if (gamepad1.left_bumper) {
                motor1.setPower(0.5);
                motor2.setPower(-0.5);
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

    public double trapezoidal_profile(double max_acceleration, double max_velocity,
                                      double distance, double elapsed_time)
    {
        //Return the current reference position based on the given motion profile times,
        //maximum acceleration, velocity, and current time.
/*
        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration))

        acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance
        cruise_dt = cruise_distance / max_velocity
        deceleration_time = acceleration_dt + cruise_dt

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deceleration_dt
        if (elapsed_time > entire_dt)
            return distance

        // if we're accelerating
        if (elapsed_time < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * elapsed_time ** 2

        // if we're cruising
  else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
            cruise_current_dt = elapsed_time - acceleration_dt

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
            cruise_distance = max_velocity * cruise_dt
            deceleration_time = elapsed_time - deceleration_time

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2
        }
*/    return 0; }

}
