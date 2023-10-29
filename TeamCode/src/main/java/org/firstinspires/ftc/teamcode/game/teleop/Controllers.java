package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Claw;

public class Controllers {

    private final CenterStage_Meet1 teleOp;
    private final Claw claw;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers(CenterStage_Meet1 teleOp, Claw claw) {
        this.teleOp = teleOp;
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

        if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
            claw.wristDown();
            claw.closeArm();
            teleOp.armTimer.reset();
            teleOp.setArmState(ArmState.PIXEL_GRAB);
        }

        if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            claw.openArm();
            teleOp.armTimer.reset();
            teleOp.setArmState(ArmState.CLAW_OPEN);
        }

        if (!currentGamepad1.a && previousGamepad1.a) {
            claw.clawSlideRunToPosition(claw.slideLow);
        }

        if (!currentGamepad1.x && previousGamepad1.x) {
            claw.clawSlideRunToPosition(claw.slideMedium);
        }

        if (!currentGamepad1.y && previousGamepad1.y) {
            claw.clawSlideRunToPosition(claw.slideMaxHeight);
        }

        if (!currentGamepad1.b && previousGamepad1.b) {
            claw.clawSlideRunToPosition(claw.slideStart);
            claw.wristDown();
            claw.openArm();
        }

        if (gamepad1.dpad_up) {
            claw.slideManualRun(claw.smallSlideManual);
        }

        if (gamepad1.dpad_down) {
            claw.slideManualRun(-claw.smallSlideManual);
        }
    }


}
