package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Claw_Meet1;

public class Controllers {

    private final CenterStage_Meet1 teleOp;
    private final Claw_Meet1 clawOld;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers(CenterStage_Meet1 teleOp, Claw_Meet1 clawOld) {
        this.teleOp = teleOp;
        this.clawOld = clawOld;
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
            clawOld.wristDown();
            clawOld.closeArm();
            teleOp.armTimer.reset();
            teleOp.setArmState(ArmState.PIXEL_GRAB);
        }

        if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            clawOld.openArm();
            teleOp.armTimer.reset();
            teleOp.setArmState(ArmState.CLAW_OPEN);
        }

        if (!currentGamepad1.a && previousGamepad1.a) {
            clawOld.clawSlideRunToPosition(clawOld.slideLow);
        }

        if (!currentGamepad1.x && previousGamepad1.x) {
            clawOld.clawSlideRunToPosition(clawOld.slideMedium);
        }

        if (!currentGamepad1.y && previousGamepad1.y) {
            clawOld.clawSlideRunToPosition(clawOld.slideMaxHeight);
        }

        if (!currentGamepad1.b && previousGamepad1.b) {
            clawOld.clawSlideRunToPosition(clawOld.slideStart);
            clawOld.wristDown();
            clawOld.openArm();
        }

        if (gamepad1.dpad_up) {
            clawOld.slideManualRun(clawOld.smallSlideManual);
        }

        if (gamepad1.dpad_down) {
            clawOld.slideManualRun(-clawOld.smallSlideManual);
        }
        if (gamepad1.start){
            clawOld.droneOpen();
        }
        if (gamepad1.back){
            clawOld.droneClose();
        }
    }


}
