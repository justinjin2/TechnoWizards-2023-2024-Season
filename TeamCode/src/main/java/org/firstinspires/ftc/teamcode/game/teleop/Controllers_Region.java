package org.firstinspires.ftc.teamcode.game.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;

public class Controllers_Region {

    private final CenterStage_Region teleOp;
    private final Intake intake;
    private final Delivery delivery;
    private final V4Bar v4Bar;
    private final Claw claw;
    private final PTO pto;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers_Region(CenterStage_Region teleOp, Intake intake, Delivery delivery,
                              V4Bar v4Bar, Claw claw, PTO pto) {
        this.teleOp = teleOp;
        this.intake = intake;
        this.delivery = delivery;
        this.v4Bar = v4Bar;
        this.claw = claw;
        this.pto = pto;
    }

    public double[][] target = {        //2d array: slideLength, 4BarPosition, clawAngle, slideAngle
            {360, 0.76, 0.54, 0},
            {340, 0.73, 0.64, 300},
            {460, 0.73, 0.67, 375},
            {510, 0.73, 0.67, 600},
    };

    public char deliveryKey = '\0';
    int clawOpenCount = 0;

    boolean droneLaunched = false;
    boolean hanging = false;
    boolean hangingToggle = false;

    public void updateCopies(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    public void readInputs(Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {

        if (gamepad1.right_trigger > 0) {
            teleOp.setDriveSpeedRatio(0.5);
        } else {
            teleOp.setDriveSpeedRatio(1.0);
        }

//--------------  Intake Keys ------------------------------------------------------
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            claw.openBothClaw();
            intake.intakeStart();
            intake.setIntakePosition(intake.intakeCenterPosition);
            teleOp.setRobotState(RobotState.INTAKE_START);
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            claw.closeBothClaw();
            intake.intakeBackSpin();
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.INTAKE_BACKSPIN);
        }

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            claw.openBothClaw();
            intake.intakeStart();
            intake.setIntakePosition(intake.the5Pixel);
            teleOp.setRobotState(RobotState.INTAKE_START);
        }

        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            intake.setIntakePositionStep(intake.theNextPixel);
        }

        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            v4Bar.setV4BarStepDown();
            deliveryKey = '\0';
        }

        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            v4Bar.setV4BarStepUp();
            deliveryKey = '\0';
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
            hanging = true;
        }

        if (currentGamepad2.start && !previousGamepad2.start) {
            delivery.droneLaunch();
            droneLaunched = true;
        }

        if (currentGamepad2.x && !previousGamepad2.x) {
            intake.intakeBackSpin();
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            delivery.resetSlideAngle();
            intake.resetMotor();

            intake.setIntakePosition(intake.intakeSafePosition);
            sleep(200);
            claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
            claw.openBothClaw();
            sleep(400);
            delivery.resetSlide();
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
            sleep(200);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            sleep(200);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);

            intake.resetMotor();
            delivery.resetSlideAngle();

            deliveryKey = '\0';
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
        if ((currentGamepad1.left_trigger > 0) || (clawOpenCount == 2)) {
            claw.openBothClaw();
            clawOpenCount = 0;
            teleOp.setRobotState(RobotState.CLAW_OPEN);
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            delivery.setSlideAngleStep(-delivery.slideAngleStep);
            claw.setClawAngleToHeight(-claw.clawAngleToHeight);
            deliveryKey = '\0';
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            intake.setIntakePosition(intake.intakeSafePosition);
            teleOp.setRobotState(RobotState.DELIVERY_START);
            deliveryKey = '1';
        }

        if (currentGamepad1.back && !previousGamepad1.back) {
            hanging = true;
            hangingToggle = !hangingToggle;
        }

        if (hangingToggle) {
            delivery.slideAngleRunToPosition(delivery.slideAngleMaxUp);
        } else if (hanging) {
            delivery.slideAngleRunToPosition(delivery.slideAngleMaxDown);
            hanging = false;
        }

        if (currentGamepad1.start && !previousGamepad1.start) {
            delivery.setHangerDeploy();
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            claw.openLeftClaw();
            clawOpenCount++;
            deliveryKey = '\0';
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            claw.openRightClaw();
            clawOpenCount++;
            deliveryKey = '\0';
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            delivery.resetSlideAngle();
            intake.resetMotor();

            intake.setIntakePosition(intake.intakeSafePosition);
            claw.openBothClaw();
            sleep(200);
            claw.setClawAnglePosition(claw.clawAngleDeliveryStage2);
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage1);
            sleep(400);
            delivery.resetSlide();
            v4Bar.setV4BarPosition(v4Bar.v4BarDownStage2);
            sleep(200);
            claw.setClawAnglePosition(claw.clawAngleIntake);
            sleep(200);
            v4Bar.setV4BarPosition(v4Bar.v4BarIntake);

            intake.resetMotor();
            delivery.resetSlideAngle();

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
            droneLaunched = false;
            intake.setIntakePosition(intake.intakeHangerPosition);
            teleOp.setRobotState(RobotState.DRONE_HANGER_START);
            deliveryKey = '\0';
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            delivery.slideRunToPositionManual_Encoder(-delivery.slideIncreaseManual);
            deliveryKey = '\0';
        }
    }
}
