package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class V4Bar {
    public ServoImplEx v4BLeft;
    public ServoImplEx v4BRight;

    public double v4BarCenterPosition = 0.5;
    public double v4BarStep = 0.01;

    public double v4BarIntake = 0.265;
    public double v4BarDownStage1 = 0.38;
    public double v4BarDownStage2 = 0.285;
    public double v4BarHangerReadyPosition = 0.38;
    public double getV4BarHangerPosition = 0.38;

    public double v4BarUpPosition = 0.55;

    public double v4BarDeliveryStage1 = 0.295;
    public double v4BarDeliveryStage2 = 0.84;

    public int v4BarUpStage1Time = 100;
    public int v4BarUpStage2Time = 280;
    public int v4BarDownTime = 200;

    public void init(HardwareMap hwmap) {
        v4BLeft = hwmap.get(ServoImplEx.class, "v4BLeft");
        v4BRight = hwmap.get(ServoImplEx.class, "v4BRight");
    }

    public void setV4BarCenterPosition() {
        v4BLeft.setPosition(v4BarCenterPosition);
        v4BRight.setPosition(v4BarCenterPosition);
    }

    public void setV4BarPosition(double position) {
        v4BLeft.setPosition(position);
        v4BRight.setPosition((position));
    }

    public void setV4BarStepDown() {
        double currentPosition1 = v4BLeft.getPosition();
        double currentPosition2 = v4BRight.getPosition();

        v4BLeft.setPosition(currentPosition1 - v4BarStep);
        v4BRight.setPosition(currentPosition2 - v4BarStep);
    }

    public void setV4BarStepUp() {
        double currentPosition1 = v4BLeft.getPosition();
        double currentPosition2 = v4BRight.getPosition();

        v4BLeft.setPosition(currentPosition1 + v4BarStep);
        v4BRight.setPosition(currentPosition2 + v4BarStep);
    }

    public double getV4BarLeftPosition() {
        return v4BLeft.getPosition();
    }

    public double getV4BarRightPosition() {
        return v4BRight.getPosition();
    }
}
