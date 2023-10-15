package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public abstract class Auto extends LinearOpMode {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;

    private TeamPropDetector propDetector;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public void setPosition(TeamPropDetector.TSEDetectorPipeline.TSEPosition position) {
        this.position = position;
    }

    public TeamPropDetector.TSEDetectorPipeline.TSEPosition getPosition() {
        return this.position;
    }

    public void startTimer() {
        elapsedTime.startTime();
    }

    public double getTimeMilliseconds() {
        return elapsedTime.milliseconds();
    }

    public void initPropDetector(PropColor color) {
        this.propDetector = new TeamPropDetector(PropColor.BLUE, hardwareMap);
    }

    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    abstract void updateTelemetry();




}
