package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.util.Utility;
public class Claw {
    private DcMotorEx clawMotorLeft;
    private DcMotorEx clawMotorRight;
    private Servo clawArm;
    private Servo clawWrist;

    private final double clawMotor_MaxVelocity = 2700; // Max speed of the deliverSlide
    public void init(HardwareMap hwMap) {
        clawMotorLeft = hwMap.get(DcMotorEx.class,"clawMotorLeft");
        clawMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotorRight = hwMap.get(DcMotorEx.class,"clawMotorRight");
        clawMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawArm = hwMap.get(Servo.class, "clawArm");
        clawWrist = hwMap.get(Servo.class, "clawWrist");

    }
    public double clawMotorLeftPosition() {
        return  clawMotorLeft.getCurrentPosition();
    }
    public double clawMotorRightPosition() {
        return  clawMotorRight.getCurrentPosition();
    }
    public void clawArmOpen() {
        clawArm.setPosition(1);
    }
    public void clawArmClose() {
        clawArm.setPosition(0);
    }
    public void clawSlideRunToPosition(int position) {

        clawMotorLeft.setTargetPosition(position);
        clawMotorRight.setTargetPosition(position);

        clawMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawMotorLeft.setVelocity(clawMotor_MaxVelocity);
        clawMotorRight.setVelocity(clawMotor_MaxVelocity);
    }

    public void resetDelivery(){
        clawMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
