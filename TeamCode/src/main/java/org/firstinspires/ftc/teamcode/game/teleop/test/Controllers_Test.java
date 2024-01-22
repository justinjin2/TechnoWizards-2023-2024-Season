package org.firstinspires.ftc.teamcode.game.teleop.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.game.teleop.test.CenterStage_Test;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

public class Controllers_Test {

    private final CenterStage_Test teleOp;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final Claw claw;

    public double[][] target = {        //2d array: slideLength, 4BarPosition, clawAngle, slideAngle
            {0,   0.7, 0.5, 0},
            {150, 0.6, 0.5, 0},
            {350, 0.6, 0.5, 100},
            {550, 0.6, 0.5, 400},
    };

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers_Test(CenterStage_Test teleOp, Intake intake, Delivery delivery,
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
        if (gamepad2.left_trigger > 0) {
            intake.setIntakePosition(intake.the5Pixel);
        }

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            intake.setIntakePositionStep(intake.theNextPixel);
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            intake.intakeStart();
            claw.openBothClaw();
            intake.setIntakePosition(intake.the5Pixel);
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            intake.intakeStop();
            intake.setIntakePosition(intake.intakeCenterPosition);
            claw.closeBothClaw();
            delivery.resetMotor();
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage1);
        }

        if (currentGamepad2.x && !previousGamepad2.x) {
            v4Bar.setV4BarPosition(v4Bar.v4BarDeliveryStage2);
        }

        //if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
        //    v4Bar.setV4BarPosition(v4Bar.v4BarIntake);
        //}

        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            claw.closeLeftClaw();
        }

        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            claw.closeRightClaw();
        }

        if (currentGamepad2.start && !previousGamepad2.start) {
            intake.setIntakePositionStep(intake.intakeStepUp);
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            intake.setIntakePositionStep(intake.intakeStepDown);
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
            delivery.slideRunToPosition_Encoder((int) target[0][0], delivery.slideRunLowVelocity);
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            delivery.slideRunToPosition_Encoder((int) target[1][0], delivery.slideRunHighVelocity);
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            delivery.slideRunToPosition_Encoder((int) target[2][0], delivery.slideRunHighVelocity);
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            delivery.slideRunToPosition_Encoder((int) target[3][0], delivery.slideRunHighVelocity);
        }

        //if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
        //    delivery.slideRunToPositionManual_Encoder(delivery.slideIncreaseManual);
        //if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
        //    delivery.slideRunToPositionManual_Encoder(-delivery.slideIncreaseManual);

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            delivery.slideAngleRunToPosition(delivery.slideAngleMaxUp);
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
        }
    }
}
