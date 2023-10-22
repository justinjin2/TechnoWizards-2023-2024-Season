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

    public final int slideStart = 0;
    public final int slideLow = 200;

    private final double clawMotor_MaxVelocity = 500; // Max speed of the deliverSlide
    public void init(HardwareMap hwMap) {
        clawMotorLeft = hwMap.get(DcMotorEx.class,"clawMotorLeft");
        clawMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotorRight = hwMap.get(DcMotorEx.class,"clawMotorRight");
        clawMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        clawArm = hwMap.get(Servo.class, "clawArm");
        clawWrist = hwMap.get(Servo.class, "clawWrist");

    }
    public double clawMotorLeftPosition() {
        return  clawMotorLeft.getCurrentPosition();
    }
    public double clawMotorRightPosition() {
        return  clawMotorRight.getCurrentPosition();
    }
    //increase means close, decrease means open
    public void openArm() {
        clawArm.setPosition(0.35);
    }
    public void closeArm() {
        clawArm.setPosition(0.46);
    }
    //decrease means down, increase means up
    public void wristDown(){
        clawWrist.setPosition(0.12);
    }
    public void wristUp(){
        clawWrist.setPosition(0.65);
    }
    public void resetArm(){clawArm.setPosition(0.5);}
    public void resetWrist(){clawWrist.setPosition(0.5);}


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
