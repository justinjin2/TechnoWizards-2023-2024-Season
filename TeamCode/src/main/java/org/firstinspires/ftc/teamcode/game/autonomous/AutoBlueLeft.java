package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.PropColor;


@Config
@Autonomous(group = "Meet One")
public class AutoBlueLeft extends Auto{

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot


        initPropDetector(PropColor.BLUE);
        initDrive();


        while (!isStarted() && !isStopRequested()) {
            setPosition(getPropDetector().getPipeline().getPosition());
            updateTelemetry();
        }

        getPropDetector().closeWebcam();
        startTimer();

        // start doing stuff
        // drive.followTrajectorySequence(getTrajectories().getBlueRight(getPosition()));

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("30 seconds count-down", 30 - (int)getTimeSeconds());
            telemetry.update();
        }


    }

    public void updateTelemetry() {
        telemetry.addData("TSE Position", getPosition().name());
        telemetry.update();
    }

}
