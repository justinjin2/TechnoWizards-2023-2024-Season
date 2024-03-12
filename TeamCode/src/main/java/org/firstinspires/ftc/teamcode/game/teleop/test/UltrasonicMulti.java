package org.firstinspires.ftc.teamcode.game.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Sensor: MB1644 multi-sensor test", group = "Sensor")
public class UltrasonicMulti extends LinearOpMode {

    AnalogInput ultrasonicLeft;
    AnalogInput ultrasonicRight;
    AnalogInput ultrasonicBackLeft;
    AnalogInput ultrasonicBackRight;

    private double voltageToInches(double voltage) {
        // Values from the linear regression calculation
        final double slope = 87.17;
        final double intercept = -12.35;

        return slope * voltage + intercept;
    }

    @Override
    public void runOpMode() {
        // Initialize all four ultrasonic sensors
        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonic_left");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonic_right");
        ultrasonicBackLeft = hardwareMap.get(AnalogInput.class, "ultrasonic_back_left"); // Corrected initialization
        ultrasonicBackRight = hardwareMap.get(AnalogInput.class, "ultrasonic_back_right"); // Initialized the new sensor

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            double voltageLeft = ultrasonicLeft.getVoltage();
            double voltageRight = ultrasonicRight.getVoltage();
            double voltageBackLeft = ultrasonicBackLeft.getVoltage();
            double voltageBackRight = ultrasonicBackRight.getVoltage();

            // Convert voltages to distances
            double distanceLeft = voltageToInches(voltageLeft);
            double distanceRight = voltageToInches(voltageRight);
            double distanceBackLeft = voltageToInches(voltageBackLeft);
            double distanceBackRight = voltageToInches(voltageBackRight);

            // Display the converted distances
            telemetry.addData("Distance Left (inches)", "%.2f", distanceLeft);
            telemetry.addData("Distance Right (inches)", "%.2f", distanceRight);
            telemetry.addData("Distance Back Left (inches)", "%.2f", distanceBackLeft);
            telemetry.addData("Distance Back Right (inches)", "%.2f", distanceBackRight);
            telemetry.update();
        }
    }
}
