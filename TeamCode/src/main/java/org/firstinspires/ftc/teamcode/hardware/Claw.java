package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    public ServoImplEx leftClaw, rightClaw;
    public ServoImplEx clawRotation;
    private DigitalChannel leftClawSensor, rightClawSensor;

    public ServoImplEx testServo;

    public double clawOpenTime = 350;
    public double clawCloseTime = 200;

    public double leftClawOpenPosition = 1.0;
    public double rightClawOpenPosition = 1.0;
    public double leftClawClosePosition = 0.31;
    public double rightClawClosePosition = 0.335;
    public double clawAngleCenter = 0.5;

    public double clawAngleIntake = 0.215;
    public double clawAngleDeliveryStage1 = 0.15;
    public double clawAngleDeliveryStage2 = 0.15; //used by 4bar return to intake
    public double clawAngleToHeight = 0.02; //steps when alide angle goes up
    public double clawAngleInit = 0.07;

    public double clawAngleHanger = 0.42;

    public int clawAngleStage1Time = 200;
    public int clawAngleStage2Time = 200;

    private double clawRotationStep = 0.01;
    private double testServoCenter = 0.5;

    public void init(HardwareMap hwmap) {
        leftClaw = hwmap.get(ServoImplEx.class, "leftClaw");
        rightClaw = hwmap.get(ServoImplEx.class, "rightClaw");
        clawRotation = hwmap.get(ServoImplEx.class, "clawRotation");
        leftClawSensor = hwmap.get(DigitalChannel.class, "leftClawSensor");
        rightClawSensor = hwmap.get(DigitalChannel.class, "rightClawSensor");

        testServo = hwmap.get(ServoImplEx.class, "testServo");

    }

    public void setTestServoCenter() { testServo.setPosition(testServoCenter);}
    public double getTestServoPosition() { return testServo.getPosition(); }

    public void setTestServoUP() {
        double currentPosition = testServo.getPosition();
        testServo.setPosition(currentPosition + clawRotationStep);
    }

    public void setTestServoDown() {
        double currentPosition = testServo.getPosition();
        testServo.setPosition(currentPosition - clawRotationStep);
    }

    public void openBothClaw() {
        leftClaw.setPosition(leftClawOpenPosition);
        rightClaw.setPosition(rightClawOpenPosition);
    }

    public void openLeftClaw() { leftClaw.setPosition(leftClawOpenPosition);}
    public void openRightClaw() { rightClaw.setPosition(rightClawOpenPosition);}

    public void closeBothClaw() {
        leftClaw.setPosition(leftClawClosePosition);
        rightClaw.setPosition(rightClawClosePosition);
    }

    public void closeLeftClaw() { leftClaw.setPosition(leftClawClosePosition);}
    public void closeRightClaw() { rightClaw.setPosition(rightClawClosePosition);}

    public void setClawAngleCenter() { clawRotation.setPosition(clawAngleCenter);}

    public void setClawAnglePosition(double position) { clawRotation.setPosition(position);}

    public void setClawRotationUp() {
        double currentPosition = clawRotation.getPosition();
        clawRotation.setPosition(currentPosition + clawRotationStep);
    }

    public void setClawRotationDown() {
        double currentPosition = clawRotation.getPosition();
        clawRotation.setPosition(currentPosition - clawRotationStep);
    }

    public void setClawAngleToHeight(double position) {
        double currentPosition = clawRotation.getPosition();
        clawRotation.setPosition(currentPosition + position);
    }

    public double getClawAngle() { return clawRotation.getPosition();}

    public boolean getLeftClawSensor() { return !leftClawSensor.getState(); }

    public boolean getRightClawSensor() { return !rightClawSensor.getState(); }
}
