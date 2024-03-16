package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.VoltageToInches;

public class Intake {

    ServoImplEx intakeRotation;
    private ColorRangeSensor leftPixelSensor, rightPixelSensor;
    private AnalogInput ultrasonicBackLeft;
    private AnalogInput ultrasonicBackRight;

    VoltageToInches voltageToInches = new VoltageToInches();

    PTO pto = new PTO();
    V4Bar v4Bar = new V4Bar();
    Claw claw = new Claw();

    double intakePower = 1.0;

    public int leftPixelDetectDistance = 15;
    public int rightPixelDetectDistance = 15;

    public double intakeInitPosition = 0.58;
    public double intakeCenterPosition = 0.5;
    public double intakeDownPosition = 0.22;
    public double intakeHangerPosition = 0.29;
    public double intakeStepUp = 0.01;
    public double intakeStepDown= -0.01;

    public double intakeSafePosition = 0.4;
    public double the5Pixel = 0.28;
    public double theNextPixel = -0.012 ;

    public int backSpinTime = 500;

    public void init(HardwareMap hwmap) {
        intakeRotation = hwmap.get(ServoImplEx.class, "intakeRotation");
        leftPixelSensor = hwmap.get(ColorRangeSensor.class, "leftPixelSensor");
        rightPixelSensor = hwmap.get(ColorRangeSensor.class, "rightPixelSensor");

        ultrasonicBackLeft = hwmap.get(AnalogInput.class, "ultrasonic_back_left"); // Corrected initialization
        ultrasonicBackRight = hwmap.get(AnalogInput.class, "ultrasonic_back_right"); // Initialized the new sensor

        pto.init(hwmap);
        v4Bar.init(hwmap);
        claw.init(hwmap);
    }

    public void setIntakeCenter() { intakeRotation.setPosition(intakeCenterPosition);}

    public void intakeStart() {
        resetMotor(); //clear encoder
        pto.motor1.setPower(intakePower);
        pto.motor2.setPower(-intakePower);
    }

    public void intakeStop() {
        pto.motor1.setPower(0);
        pto.motor2.setPower(0);

        resetMotor(); //clear encoder
    }

    public void intakeBackSpin() {
        pto.motor1.setPower(-intakePower);
        pto.motor2.setPower(intakePower);
    }

    public void setIntakePosition(double position) {
        intakeRotation.setPosition(position);
    }

    public void setIntakePositionStep(double position) {
        double currentPosition = intakeRotation.getPosition();
        intakeRotation.setPosition(currentPosition + position);
    }

    public void resetMotor() {

        pto.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pto.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pto.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pto.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getIntakeDownPosition() { return intakeRotation.getPosition();}

    public double getMotor1Current() { return pto.motor1.getCurrent(AMPS);}

    public double getMotor2Current() { return pto.motor2.getCurrent(AMPS);}

    public double getLeftPixelSensor() { return leftPixelSensor.getDistance(DistanceUnit.MM); }

    public double getRightPixelSensor() { return rightPixelSensor.getDistance(DistanceUnit.MM); }

    public double getUltrasonicBackLeft() { return voltageToInches.calculateInches(ultrasonicBackLeft.getVoltage()); }

    public double getUltrasonicBackRight() { return voltageToInches.calculateInches(ultrasonicBackRight.getVoltage()); }
}
