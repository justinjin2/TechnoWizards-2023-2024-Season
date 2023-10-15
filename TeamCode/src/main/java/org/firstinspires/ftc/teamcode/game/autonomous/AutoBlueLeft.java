package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Config
@Autonomous(group = "Meet One")
public class AutoBlueLeft extends Auto{

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot

        while (!isStarted()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();
        startTimer();

        while (!isStopRequested()) {
            updateTelemetry();
        }


    }


    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
