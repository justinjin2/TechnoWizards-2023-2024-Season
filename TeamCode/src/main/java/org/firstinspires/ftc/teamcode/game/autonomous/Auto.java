package org.firstinspires.ftc.teamcode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.vision.PropColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class Auto extends LinearOpMode {

    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    static final double DESIRED_DISTANCE = 8.0;

    // ----- Vision ----- //

    private TeamPropDetector.TSEDetectorPipeline.TSEPosition position;
    private TeamPropDetector propDetector;



    // ------ Hardware ------ //

    public SampleMecanumDrive drive;

    public Claw claw;

    // ----- Movement ------ //

    private Trajectories trajectories;

    // ----- Elapsed Time ---- //

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // ----- April Tag ----- //

    static final String webcamName = "Webcam 1";

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    public int backboardTagID =    -1;
    public double  rangeError =       0;
    public double  headingError =     0;
    public double  yawError =         0;
    public double speed =             0;
    public double turn =              0;
    public double strafe =            0;



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

    public ElapsedTime getLoopTimer() {
        return loopTimer;
    }

    public ElapsedTime getAutoTimer() {
        return autoTimer;
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
        this.trajectories = new Trajectories(drive, claw);
    }


    public int getSecondsLeft() {
        return (int) (30 - elapsedTime.seconds());
    }

    public TeamPropDetector getPropDetector() {
        return propDetector;
    }

    public abstract void updateTelemetry();

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void setVisionPortal(VisionPortal visionPortal) {
        this.visionPortal = visionPortal;
    }

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public void setAprilTag(AprilTagProcessor aprilTag) {
        this.aprilTag = aprilTag;
    }

    public AprilTagDetection getDesiredTag() {
        return desiredTag;
    }

    public void setDesiredTag(AprilTagDetection desiredTag) {
        this.desiredTag = desiredTag;
    }
}
