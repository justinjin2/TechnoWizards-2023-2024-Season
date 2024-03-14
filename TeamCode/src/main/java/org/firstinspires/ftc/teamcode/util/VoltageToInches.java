package org.firstinspires.ftc.teamcode.util;

public class VoltageToInches {
    public double calculateInches(double voltage) {
        // Values from the linear regression calculation
        final double slope = 87.17;
        final double intercept = -12.35;

        return slope * voltage + intercept;
    }
}
