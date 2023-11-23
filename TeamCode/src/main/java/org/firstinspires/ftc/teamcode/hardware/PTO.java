package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PTO {
    public DcMotorEx motor1;
    public DcMotorEx motor2;

public void init(HardwareMap hwMap){
    motor1 = hwMap.get(DcMotorEx.class, "motor1");
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor1.setDirection(DcMotorEx.Direction.FORWARD);

    motor2 = hwMap.get(DcMotorEx.class, "motor2");
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setDirection(DcMotorEx.Direction.REVERSE);
}

}
