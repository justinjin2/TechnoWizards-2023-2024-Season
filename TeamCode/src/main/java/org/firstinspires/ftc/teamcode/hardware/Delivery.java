package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Delivery {
    ElapsedTime deliveryTimeout;
    ElapsedTime slideReturnTimeOut;

    DcMotorEx slideRotation;
    public ServoImplEx droneLauncher;
    public DigitalChannel redLED;
    public DigitalChannel greenLED;
    private DigitalChannel leftSlideSensor, rightSlideSensor;
    private DigitalChannel slideAngleSensor;

    PTO pto = new PTO();
    V4Bar v4Bar = new V4Bar();
    Claw claw = new Claw();

    public int slideRunHighVelocity = 2800;
    public int slideReturnVelocity = 1400;
    public int slideRunLowVelocity = 1000;
    public int slideAngleVelocity = 2800;

    public int slideMaxExtend = 600;
    public int slideStart = 0;
    public int slideIncreaseManual = 50;

    public int slideAngleMaxUp = 900;
    public int slideAngleMaxDown = 0;
    public int slideAngleStep = 100;

    public int slideExtendDrone = 480;
    public int slideAngleDrone = 700;

    public double droneInit = 0.5;
    public double droneLaunch = 1.0;

    public static double p1 = 0.005, i1 = 0.002, d1 = 0.0004;
    public static double f1 = 0;

    public static double p2 = 0.005, i2 = 0.002, d2 = 0.0004;
    public static double f2 = 0;

    double power1 = 0, power2 = 0;
    int position1 = 0, position2 = 0;

    public static double KV = 0, KA = 0, KStatic = 0;

    private PIDCoefficients coffes1 = new PIDCoefficients(p1, i1, d1);
    private PIDCoefficients coffes2 = new PIDCoefficients(p2, i2, d2);

    private PIDFController controller1 = new PIDFController(coffes1, KV, KA, KStatic);
    private PIDFController controller2 = new PIDFController(coffes2, KV, KA, KStatic);

    public void init(HardwareMap hwmap) {
        slideRotation = hwmap.get(DcMotorEx.class, "slideRotation");
        slideRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneLauncher = hwmap.get(ServoImplEx.class, "droneLauncher");

        leftSlideSensor = hwmap.get(DigitalChannel.class, "leftSlideSensor");
        rightSlideSensor = hwmap.get(DigitalChannel.class, "rightSlideSensor");
        slideAngleSensor = hwmap.get(DigitalChannel.class, "slideAngleSensor");

        redLED = hwmap.get(DigitalChannel.class, "redLED");
        greenLED = hwmap.get(DigitalChannel.class, "greenLED");

        pto.init(hwmap);
        v4Bar.init(hwmap);
        claw.init(hwmap);

        droneLauncher.setPosition(droneInit);

        slideReturnTimeOut = new ElapsedTime();
    }

    public void slideRunToTarget_PID(double position) {

        controller1.setTargetPosition(position);
        controller2.setTargetPosition(position);

        position1 = pto.motor1.getCurrentPosition();
        position2 = pto.motor2.getCurrentPosition();

        double pid1 = controller1.update(position1);
        double pid2 = controller2.update(position2);
        power1 = pid1 + f1;
        power2 = pid2 + f2;

        pto.motor1.setPower(power1);
        pto.motor2.setPower(power2);
    }

    public void slideRunToPosition_Encoder(int position, int velocity) {
        pto.motor1.setTargetPosition(position);
        pto.motor2.setTargetPosition(position);
        pto.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pto.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pto.motor1.setVelocity(velocity);
        pto.motor2.setVelocity(velocity);

    }

    public void slideRunToPositionManual_Encoder(int incrementPosition) {

        int currentPosition1, currentPosition2, meanPosition;

        currentPosition1 = pto.motor1.getCurrentPosition();
        currentPosition2 = pto.motor2.getCurrentPosition();
        meanPosition = (Math.abs(currentPosition1) + Math.abs(currentPosition2)) / 2;

        if (incrementPosition > 0) {
            if ((meanPosition + incrementPosition) < slideMaxExtend) {
                pto.motor1.setTargetPosition(currentPosition1 + incrementPosition);
                pto.motor2.setTargetPosition(currentPosition2 + incrementPosition);
            } else {
                pto.motor1.setTargetPosition(slideMaxExtend);
                pto.motor2.setTargetPosition(slideMaxExtend);

            }
        }
        if (incrementPosition < 0) {
            if ((meanPosition + incrementPosition) > slideStart) {
                pto.motor1.setTargetPosition(currentPosition1 + incrementPosition);
                pto.motor2.setTargetPosition(currentPosition2 + incrementPosition);
            }
            else {
                pto.motor1.setTargetPosition(slideStart);
                pto.motor2.setTargetPosition(slideStart);
            }
        }
        pto.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pto.motor1.setVelocity(slideRunHighVelocity);
        pto.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pto.motor2.setVelocity(slideRunHighVelocity);
    }

    public void slideAngleRunToPosition(int position) {
        slideRotation.setTargetPosition(position);
        slideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotation.setVelocity(slideAngleVelocity);
    }

    public void setSlideAngleStep(int increment) {
        int currentPosition;

        currentPosition = slideRotation.getCurrentPosition();
        if (increment > 0) {
            if ((currentPosition + increment) < slideAngleMaxUp) {
                slideRotation.setTargetPosition(currentPosition + increment);
            } else {
                slideRotation.setTargetPosition(slideAngleMaxUp);
            }
        }
        if (increment < 0) {
            if ((currentPosition + increment) > slideAngleMaxDown) {
                slideRotation.setTargetPosition(currentPosition + increment);
            }
            else {
                slideRotation.setTargetPosition(slideAngleMaxDown);
            }
        }
        slideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotation.setVelocity(slideAngleVelocity);
    }

    public void resetMotor() {
        pto.motor1.setPower(0);
        pto.motor2.setPower(0);

        slideRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotation.setTargetPosition(slideAngleMaxDown);
        slideRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotation.setVelocity(slideAngleVelocity);

        //prepare for delivery mode
        pto.motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pto.motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pto.motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pto.motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlideAngle() {
        slideRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlide() {
        slideReturnTimeOut.reset();

        while (((leftSlideSensor.getState()) && (rightSlideSensor.getState())) &&
                (slideReturnTimeOut.milliseconds() < 1500)) {
            pto.motor1.setPower(-0.5);
            pto.motor2.setPower(-0.5);
        }

        slideReturnTimeOut.reset();
        while ((slideAngleSensor.getState()) && (slideReturnTimeOut.milliseconds() < 1000)) {
            slideRotation.setPower(-0.5);
        }
        resetMotor();
    }

    public void droneInit() { droneLauncher.setPosition(droneInit); }

    public void droneLaunch() {droneLauncher.setPosition(droneLaunch);}

    public int getMotor1Position() {
        return pto.motor1.getCurrentPosition();
    }

    public int getMotor2Position() {
        return pto.motor2.getCurrentPosition();
    }

    public int getSlideAnglePosition() {
        return slideRotation.getCurrentPosition();
    }

    public double getSlideAngleMotorCurrent() { return slideRotation.getCurrent(AMPS);}

    public boolean getLeftSlideSensor() { return !leftSlideSensor.getState(); }

    public boolean getRightSlideSensor() { return !rightSlideSensor.getState(); }

    public boolean getSlideAngleSensor() { return !slideAngleSensor.getState(); }

}
