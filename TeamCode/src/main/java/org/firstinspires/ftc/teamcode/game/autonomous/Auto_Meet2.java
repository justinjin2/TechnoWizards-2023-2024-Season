package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public abstract class Auto_Meet2 extends LinearOpMode {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;
    private TeamPropDetector propDetector;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public SampleMecanumDrive drive;

    public Intake intake;
    public Delivery delivery;
    public V4Bar v4Bar;
    public PTO pto;

    public static int autoDeliveryPosition = 400;
    public static int slideHomePosition = 0;

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
        this.intake = new Intake();
        this.delivery = new Delivery();
        this.v4Bar = new V4Bar();
        this.pto = new PTO();

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
    }


    public int getSecondsLeft() {
        return (int) (30 - elapsedTime.seconds());
    }

    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    abstract void updateTelemetry();

}
