package org.firstinspires.ftc.teamcode.game.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Slide;

public class Controllers {

    private final CenterStage_Meet2 teleOp;
    private final Slide slide;

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();


    public Controllers(CenterStage_Meet2 teleOp, Slide slide) {
        this.teleOp = teleOp;
        this.slide = slide;
    }

    public void updateCopies(Gamepad gamepad1, Gamepad gamepad2) {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    public void readInputs(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.start){
            slide.runToPosition(slide.slideTest);
        }
        if (gamepad1.back){
            slide.runToPosition(slide.slideStart);
        }
    }


}
