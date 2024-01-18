package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

public class Controllers_V1 {

    private final CenterStage_Meet3_V1 teleOp;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final Claw claw;
    private final PTO pto;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers_V1(CenterStage_Meet3_V1 teleOp, Intake intake, Delivery delivery,
                          V4Bar v4Bar, Claw claw, PTO pto) {
        this.teleOp = teleOp;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.claw = claw;
        this.pto = pto;
    }

    public double[][] target = {        //2d array: slideLength, 4BarPosition, clawAngle, slideAngle
            {375, 0.76, 0.54, 0},
            {375, 0.73, 0.64, 160},
            {510, 0.73, 0.67, 318},
    };

    public char deliveryKey = '\0';

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
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            claw.openBothClaw();
            intake.intakeStart();
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.INTAKE_START);
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            claw.closeBothClaw();
            intake.intakeBackSpin();
            teleOp.setRobotState(RobotState.INTAKE_BACKSPIN);
        }

//        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//            intake.setIntakePosition(intake.intakeSafePosition);
//            claw.setClawAnglePosition(claw.clawAngleIntake);
//            claw.openBothClaw();
//        }
//
//        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//            intake.setIntakePosition(intake.intakeSafePosition);
//            claw.setClawAnglePosition(claw.clawAngleIntake);
//            claw.openBothClaw();
//            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
//        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            intake.setIntakePositionStep(intake.intakeStepDown);
        }

        if (currentGamepad2.start && !previousGamepad2.start) {
            delivery.droneLaunch();
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            intake.intakeBackSpin();
        }
        if (currentGamepad2.x && !previousGamepad2.x) {
            intake.intakeStop();
            delivery.resetMotor();
        }

        if (currentGamepad2.y && !previousGamepad2.y) {
            claw.openBothClaw();
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            claw.closeBothClaw();
        }

//        if (currentGamepad2.right_trigger > 0) {
//            delivery.droneLaunch();
//        }

//-------------- Delivery Keys -----------------------------------------------------
        if (currentGamepad1.left_trigger > 0) {
            claw.openBothClaw();
            teleOp.setRobotState(RobotState.CLAW_OPEN);
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            delivery.setSlideAngleStep(delivery.slideAngleStep);
            claw.setClawAngleToHeight(claw.clawAngleToHeight);
            deliveryKey = '\0';
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            delivery.setSlideAngleStep(-delivery.slideAngleStep);
            claw.setClawAngleToHeight(-claw.clawAngleToHeight);
            deliveryKey = '\0';
        }

        if (currentGamepad1.back && !previousGamepad1.back) {
            v4Bar.setV4BarStepDown();
            deliveryKey = '\0';
        }

        if (currentGamepad1.start && !previousGamepad1.start) {
            v4Bar.setV4BarStepUp();
            deliveryKey = '\0';
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            claw.setClawRotationUp();
            deliveryKey = '\0';
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            claw.setClawRotationDown();
            deliveryKey = '\0';
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            intake.setIntakePosition(intake.intakeSafePosition);
            delivery.slideAngleRunToPosition((delivery.slideAngleMaxDown));
            delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideReturnVelocity);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            deliveryKey = '\0';
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            deliveryKey = 'a';
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            deliveryKey = 'x';
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            deliveryKey = 'y';
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            delivery.slideRunToPositionManual_Encoder(delivery.slideIncreaseManual);
            deliveryKey = '\0';
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            delivery.slideRunToPositionManual_Encoder(-delivery.slideIncreaseManual);
            deliveryKey = '\0';
        }
    }
}
