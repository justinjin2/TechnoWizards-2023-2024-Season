package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public abstract class Auto extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;
    private TeamPropDetector propDetector;

    private final ElapsedTime elapsedTime = new ElapsedTime();


    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime generalTimer = new ElapsedTime();



    public SampleMecanumDrive drive;

    public Intake intake;
    public Delivery delivery;
    public V4Bar v4Bar;
    public PTO pto;
    public Claw claw;

    public static int DELIVER_POSITION = 200;

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

    /**
     * Starts the timer that should run for the whole game
     */
    public void startTimer() {
        elapsedTime.reset();
        elapsedTime.startTime();
    }

    /**
     * Gets the time
     * @return time left in the game in seconds
     */
    public double getTimeSeconds() {
        return elapsedTime.seconds();
    }

    public void initPropDetector(PropColor color) {
        this.propDetector = new TeamPropDetector(color, hardwareMap);
    }

    public  void initDrive() {
        this.drive = new SampleMecanumDrive(hardwareMap);

        this.intake = new Intake();
        this.delivery = new Delivery();
        this.v4Bar = new V4Bar();
        this.pto = new PTO();
        this.claw = new Claw();

        this.trajectories = new Trajectories(drive);

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);
    }

    public int getSecondsLeft() {
        return (int) (30 - elapsedTime.seconds());
    }

    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    abstract void updateTelemetry();

    public ElapsedTime getLoopTimer() {
        return loopTimer;
    }

    public ElapsedTime getGeneralTimer() {
        return generalTimer;
    }
}
