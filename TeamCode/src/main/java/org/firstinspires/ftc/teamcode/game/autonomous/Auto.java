package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public abstract class Auto extends LinearOpMode {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;
    private TeamPropDetector propDetector;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public SampleMecanumDrive drive;

    public Claw claw;

    private Trajectories trajectories;



    public void setPosition(TeamPropDetector.TSEDetectorPipeline.TSEPosition position) {
        this.position = position;
    }

    public TeamPropDetector.TSEDetectorPipeline.TSEPosition getPosition() {
        return this.position;
    }

    public Trajectories getTrajectories() {
        return trajectories;
    }

    public void startTimer() {
        elapsedTime.reset();
        elapsedTime.startTime();
    }

    public double getTimeSeconds() {
        return elapsedTime.seconds();
    }

    public void initPropDetector(PropColor color) {
        this.propDetector = new TeamPropDetector(color, hardwareMap);
    }

    public  void initDrive() {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.claw = new Claw();
        claw.init(hardwareMap);
        //this.trajectories = new Trajectories(drive);
    }


    public int getSecondsLeft() {
        return (int) (30 - elapsedTime.seconds());
    }

    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    abstract void updateTelemetry();

}
