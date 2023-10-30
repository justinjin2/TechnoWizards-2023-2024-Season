package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private DcMotorEx clawMotorLeft;
    private DcMotorEx clawMotorRight;
    private Servo clawArm;
    private Servo clawWrist;

    public final int slideStart = 0;
    public final int slideLow = 1800;
    public final int slideMedium = 2800;
    public final int slideMaxHeight = 4000;
    public int slideManual = 250;
    public int smallSlideManual = 150;

    // --- Timings --- //
    public final double armCloseTime = 200;
    public final double armOpenTime = 100;
    public final double armWristUpTime = 150;
    public final double slideHalfDownTime = 1000;

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

        clawMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

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
        clawArm.setPosition(0.33);
    }
    public void closeArm() {
        clawArm.setPosition(0.49);
    }
    //decrease means down, increase means up
    public void wristDown(){
        clawWrist.setPosition(0.15);
    }
    public void wristUp(){
        clawWrist.setPosition(0.70);
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
    public void slideManualRun(int incrementPosition) {
        int currentPositionLeft, currentPositionRight, meanPosition;

        currentPositionLeft = clawMotorLeft.getCurrentPosition();
        currentPositionRight = clawMotorRight.getCurrentPosition();
        meanPosition = (Math.abs(currentPositionLeft) + Math.abs(currentPositionRight)) / 2;

        if (incrementPosition > 0) {
            if ((meanPosition + incrementPosition) < slideMaxHeight) {
                clawMotorLeft.setTargetPosition(currentPositionLeft + incrementPosition);
                clawMotorRight.setTargetPosition(currentPositionRight + incrementPosition);
            } else {
                clawMotorLeft.setTargetPosition(slideMaxHeight);
                clawMotorRight.setTargetPosition(slideMaxHeight);
            }
        }
        if (incrementPosition < 0) {
            if ((meanPosition + incrementPosition) > slideStart) {
                clawMotorLeft.setTargetPosition(currentPositionLeft + incrementPosition);
                clawMotorRight.setTargetPosition(currentPositionRight + incrementPosition);
            } else {
                clawMotorLeft.setTargetPosition(slideStart);
                clawMotorRight.setTargetPosition(slideStart);
            }
        }
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
