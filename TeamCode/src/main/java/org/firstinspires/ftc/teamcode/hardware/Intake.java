package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    ServoImplEx intakeRotation;
    private  DigitalChannel leftPixelSensor, rightPixelSensor;
    PTO pto = new PTO();
    V4Bar v4Bar = new V4Bar();
    Claw claw = new Claw();

    double intakePower = 1.0;

    public double intakeInitPosition = 0.55;
    public double intakeCenterPosition = 0.5;
    public double intakeDownPosition = 0.26;
    public double intakeStepUp = -0.01;
    public double intakeStepDown= 0.01;

    public double intakeSafePosition = 0.4;

    public int backSpinTime = 500;

    public void init(HardwareMap hwmap) {
        intakeRotation = hwmap.get(ServoImplEx.class, "intakeRotation");
        leftPixelSensor = hwmap.get(DigitalChannel.class, "leftPixelSensor");
        rightPixelSensor = hwmap.get(DigitalChannel.class, "rightPixelSensor");
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

    public boolean getLeftPixelSensor() { return !leftPixelSensor.getState(); }

    public boolean getRightPixelSensor() { return !rightPixelSensor.getState(); }
}
