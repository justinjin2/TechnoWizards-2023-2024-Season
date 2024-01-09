package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

public class Controllers {

    private final CenterStage_Meet3 teleOp;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final Claw claw;
    private final PTO pto;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers(CenterStage_Meet3 teleOp, Intake intake, Delivery delivery,
                       V4Bar v4Bar, Claw claw, PTO pto) {
        this.teleOp = teleOp;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.claw = claw;
        this.pto = pto;
    }

    public double[][] target = {        //2d array: slideLength, 4BarPosition, clawAngle, slideAngle
            {200, 0.77, 0.50, 100},
            {350, 0.77, 0.50, 200},
            {550, 0.77, 0.50, 300},
    };

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
            claw.openBothClaw();
            intake.intakeStart();
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.INTAKE_START);
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
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
            intake.setIntakePositionStep(intake.intakeStepUp);
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            intake.intakeBackSpin();
        }

        if (currentGamepad2.y && !previousGamepad2.y) {
            claw.openBothClaw();
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            claw.closeBothClaw();
        }

//-------------- Delivery Keys -----------------------------------------------------
        if (currentGamepad1.left_trigger > 0) {
            claw.openBothClaw();
            teleOp.setRobotState(RobotState.CLAW_OPEN);
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            delivery.setSlideAngleStep(delivery.slideAngleStep);
            claw.setClawAngleToHeight(claw.clawAngleToHeight);
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            delivery.setSlideAngleStep(-delivery.slideAngleStep);
            claw.setClawAngleToHeight(-claw.clawAngleToHeight);
        }

        if (currentGamepad1.back && !previousGamepad1.back) {
            v4Bar.setV4BarStepDown();
        }

        if (currentGamepad1.start && !previousGamepad1.start) {
            v4Bar.setV4BarStepUp();
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            claw.setClawRotationUp();
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            claw.setClawRotationDown();
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            intake.setIntakePosition(intake.intakeSafePosition);
            delivery.slideRunToPosition_Encoder(delivery.slideStart, delivery.slideReturnVelocity);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            delivery.slideAngleRunToPosition((delivery.slideAngleMaxDown));
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            delivery.slideRunToPosition_Encoder((int) target[0][0], delivery.slideRunHighVelocity);
            v4Bar.setV4BarPosition(target[0][1]);
            claw.setClawAnglePosition(target[0][2]);
            delivery.slideAngleRunToPosition((int)target[0][3]);
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            delivery.slideRunToPosition_Encoder((int) target[1][0], delivery.slideRunHighVelocity);
            v4Bar.setV4BarPosition(target[1][1]);
            claw.setClawAnglePosition(target[1][2]);
            delivery.slideAngleRunToPosition((int)target[1][3]);
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            delivery.slideRunToPosition_Encoder((int) target[2][0], delivery.slideRunHighVelocity);
            v4Bar.setV4BarPosition(target[2][1]);
            claw.setClawAnglePosition(target[2][2]);
            delivery.slideAngleRunToPosition((int)target[2][3]);
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            delivery.slideRunToPositionManual_Encoder(delivery.slideIncreaseManual);

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            delivery.slideRunToPositionManual_Encoder(-delivery.slideIncreaseManual);
    }
}
