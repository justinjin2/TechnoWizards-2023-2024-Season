package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Intake {
    PTO pto = new PTO();

    private ServoImplEx stopper1;
    private ServoImplEx stopper2;
    private ServoImplEx heightAdjust;

    public void init(HardwareMap hwMap){
        stopper1 = hwMap.get(ServoImplEx.class, "stopper1");
        stopper2 = hwMap.get(ServoImplEx.class, "stopper2");
        heightAdjust = hwMap.get(ServoImplEx.class, "heightAdjust");
    }
    public void start (){
        pto.motor1.setPower(1);
        pto.motor2.setPower(1);
    }
    public void stop (){
        pto.motor1.setPower(0);
        pto.motor2.setPower(0);
    }
    public void reverse (){
        pto.motor1.setPower(-1);
        pto.motor2.setPower(-1);
    }
}
