package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

@Config
@Autonomous(group = "Meet One")
public class AutoBlueLeft extends LinearOpMode {


    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;

    @Override
    public void runOpMode() throws InterruptedException {

        // Do hardware stuff
        // initialize robot

        TeamPropDetector teamPropDetector =  new TeamPropDetector(PropColor.BLUE, hardwareMap);

        position = teamPropDetector.getPipeline().getPosition();

        waitForStart();

        teamPropDetector.closeWebcam();

        while (!isStopRequested()) {
            updateTelemetry();
        }

    }


    private void updateTelemetry() {
        telemetry.addData("TSE Position", position.name());
        telemetry.update();
    }




}
