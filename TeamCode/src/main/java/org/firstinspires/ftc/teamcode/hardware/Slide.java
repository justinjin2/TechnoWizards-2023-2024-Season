package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    PTO pto = new PTO();

    public final int slideTest = 700;
    public final int slideStart = 0;
    public void init(HardwareMap hwMap) {
        pto.init(hwMap);
    }
    public void runToPosition(int position) {

        pto.motor1.setTargetPosition(position);
        pto.motor2.setTargetPosition(position);

        pto.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pto.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pto.motor1.setPower(1);
        pto.motor2.setPower(1);
    }
    public double motor1Position() {
        return pto.motor1.getCurrentPosition();
    }
    public double motor2Position() {
        return pto.motor2.getCurrentPosition();
    }

}
