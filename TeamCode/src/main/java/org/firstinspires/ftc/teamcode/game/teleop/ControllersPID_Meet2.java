package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

public class ControllersPID_Meet2 {

    private final CenterStagePID_Meet2 teleOp;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final Claw claw;

    boolean delivery_mode = false;
    boolean intake_mode = true;

    public double[][] target = {        //2d array: slideLength, 4BarPosition, clawAngle, slideAngle
            {0,   0.7, 0.5, 0},
            {150, 0.6, 0.5, 0},
            {350, 0.6, 0.5, 100},
            {550, 0.6, 0.5, 400},
    };

    double targetPID = 0;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public ControllersPID_Meet2(CenterStagePID_Meet2 teleOp, Intake intake, Delivery delivery,
                                V4Bar v4Bar, Claw claw) {
        this.teleOp = teleOp;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.claw = claw;
    }

    public void updateCopies(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    public void readInputs(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.right_trigger > 0) {
            teleOp.setDriveSpeedRatio(0.35);
        } else {
            teleOp.setDriveSpeedRatio(1.0);
        }

//--------------  Intake Keys ------------------------------------------------------
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            intake.intakeStart();
            delivery_mode = false;
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            intake.intakeStop();
            delivery.resetMotor();
            delivery_mode = false;
            targetPID = target[0][0];
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            intake.setIntakePosition(intake.intakeDownPosition);
            delivery_mode = false;
        }

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            intake.setIntakePosition(intake.intakeCenterPosition);
            delivery_mode = false;
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            intake.setIntakePositionStep(intake.intakeStepDown);
            delivery_mode = false;
        }

        if (currentGamepad2.start && !previousGamepad2.start) {
            intake.setIntakePositionStep(intake.intakeStepUp);
            delivery_mode = false;
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            intake.intakeBackSpin();
            delivery_mode = false;
        }

//-------------- Delivery Keys -----------------------------------------------------
/*        if ((target == 0) && (Math.abs(delivery.getMotor1Position()) <= 2)) {
            delivery_state = false;
            //delivery.deliveryReset(); //need condition to finish delivery,read motor take long time
        }
*/
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            delivery.setSlideAngleStep(delivery.slideAngleStep);
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            delivery.setSlideAngleStep(-delivery.slideAngleStep);
        }

        if (currentGamepad1.back && !previousGamepad1.back) {
            v4Bar.setV4BarStepDown();
            claw.setClawRotationDown();

        }

        if (currentGamepad1.start && !previousGamepad1.start) {
            v4Bar.setV4BarStepUp();
            claw.setClawRotationUp();
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            claw.setClawRotationUp();
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            claw.setClawRotationDown();
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            targetPID = target[0][0];
            delivery_mode = true;
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            targetPID = target[1][0];
            delivery_mode = true;
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            targetPID = target[2][0];
            delivery_mode = true;
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            targetPID = target[3][0];
            delivery_mode = true;
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            if ((targetPID + delivery.slideIncreaseManual) < delivery.slideMaxExtend) {
                targetPID += delivery.slideIncreaseManual;
            } else {
                targetPID = delivery.slideMaxExtend;
            }
            delivery_mode = true;
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            if ((targetPID - delivery.slideIncreaseManual) > delivery.slideStart) {
                targetPID -= delivery.slideIncreaseManual;
            } else {
                targetPID = delivery.slideStart;
            }
            delivery_mode = true;
        }

        if (delivery_mode) {
            delivery.slideRunToTarget_PID(targetPID);
        }
    }
}
