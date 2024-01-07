package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PTO {

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public void init(HardwareMap hwmap) {
        motor1 = hwmap.get(DcMotorEx.class, "motor1");
        motor2 = hwmap.get(DcMotorEx.class, "motor2");

        //motor1.setDirection(DcMotorEx.Direction.REVERSE);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
