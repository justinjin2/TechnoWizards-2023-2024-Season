package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.game.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.PTO;
import org.firstinspires.ftc.teamcode.hardware.V4Bar;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public abstract class Auto extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ------- Team Prop ------- //
    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;
    private TeamPropDetector propDetector;


    // ------- Timers ------- //
    private final ElapsedTime elapsedTime = new ElapsedTime();
    public final ElapsedTime loopTimer = new ElapsedTime();
    public final ElapsedTime generalTimer = new ElapsedTime();
    public final ElapsedTime clawOpenTimer = new ElapsedTime();

    public static double CLAW_OPEN_TIME = 350;


    // ------- Drive ------- //
    public SampleMecanumDrive drive;
    private Trajectories trajectories;
    public RobotState robotState;

    // ------- Hardware ------- //
    public Intake intake;
    public Delivery delivery;
    public V4Bar v4Bar;
    public PTO pto;
    public Claw claw;

    // ------- Constants ------- //
    public static int DELIVER_POSITION = 200;
    public static int SLIDE_POSITION_ONE = 200;
    public static int SLIDE_POSITION_TWO = 410;
    public static double V4BAR_DELIVERY = 0.82;
    public static double CLAW_DELIVERY = 0.41;

    /**ion of the prop after we start the game, to retrieve that value
     * when getting the robot's trajectory
     *
     * @param position The position that is retrieved from getPosition()y = 0.82;
     *     public static double clawDelivery = .44;
     */
    public void setPosition(TeamPropDetector.TSEDetectorPipeline.TSEPosition position) {
        this.position = position;
    }

    /**
     * Gets the position of the Team Prop Detector Pipeline
     * @return TSEPosition Enum
     */
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


    /**
     * Starts the prop detector pipeline, which will open the camera
     * @param color Color to be detected (RED or BLUE)
     */
    public void initPropDetector(PropColor color) {
        this.propDetector = new TeamPropDetector(color, hardwareMap);
    }

    /**
     * Initializes all important hardware an drive
     * This includes SampleMecanumDrive
     */
    public  void initDrive() {
        this.drive = new SampleMecanumDrive(hardwareMap);

        this.intake = new Intake();
        this.delivery = new Delivery();
        this.v4Bar = new V4Bar();
        this.pto = new PTO();
        this.claw = new Claw();

        this.trajectories = new Trajectories(drive, intake, delivery, v4Bar, pto, claw);

        intake.init(hardwareMap);
        delivery.init(hardwareMap);
        v4Bar.init(hardwareMap);
        claw.init(hardwareMap);
        pto.init(hardwareMap);
    }

    /**
     * Subtracts 30 from timer to get time left in autonomous
     * @return seconds left in the game
     */
    public int getSecondsLeft() {
        return (int) (30 - elapsedTime.seconds());
    }

    /**
     * @return the prop detector, which can be used to get the location of the team element
     */
    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    /**
     * Updates the telemetry
     */
    abstract void updateTelemetry();
}
