package com.team9889.ftc2019.subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2019.Constants;
import com.team9889.lib.android.FileWriter;
import com.team9889.lib.hardware.Motor;
import com.team9889.lib.hardware.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;


/**
 * Created by Eric on 7/26/2019.
 */

public class Robot{

    // public WebcamName webcam;

    public Motor fLDrive, fRDrive, bLDrive, bRDrive;
    public RevIMU imu = null;

    public Motor intakeLeft, intakeRight;

    public Motor flyWheel;
    public Servo fwArm;

    public Servo wgGrabber, wgLeft, wgRight;

    public boolean redAuto;

    public RevBulkData bulkDataMaster, bulkDataSlave;
    private ExpansionHubEx revHubMaster, revHubSlave;

    public HardwareMap hardwareMap;

    private static Robot mInstance = null;

    public static Robot getInstance() {
        if (mInstance == null)
            mInstance = new Robot();

        return mInstance;
    }

    private MecanumDrive mMecanumDrive = new MecanumDrive();
    private Intake mIntake = new Intake();
    private FlyWheel mFW = new FlyWheel();
    private WobbleGoal mWG = new WobbleGoal();
    private Camera mCamera = new Camera();

    // List of subsystems
    private List<Subsystem> subsystems = Arrays.asList(mMecanumDrive, mIntake, mFW, mWG, mCamera);

    public void init(HardwareMap hardwareMap, boolean auto){
        this.hardwareMap = hardwareMap;

        Date currentData = new Date();
        SimpleDateFormat format = new SimpleDateFormat("dd.M.yyyy hh:mm:ss");

        RobotLog.a("Robot Init Started at " + format.format(currentData));

        // Rev Hubs
        revHubMaster = hardwareMap.get(ExpansionHubEx.class, Constants.kRevHubMaster);
        revHubSlave = hardwareMap.get(ExpansionHubEx.class, Constants.kRevHubSlave);

        // Camera
        // webcam = hardwareMap.get(WebcamName.class, Constants.kWebcam);

        // Drive
        fLDrive = new Motor(hardwareMap, Constants.DriveConstants.kLeftDriveMasterId, 1,
                DcMotorSimple.Direction.FORWARD, true, true, true);
        bLDrive = new Motor(hardwareMap, Constants.DriveConstants.kLeftDriveSlaveId, 1,
                DcMotorSimple.Direction.FORWARD, true, true, true);
        fRDrive = new Motor(hardwareMap, Constants.DriveConstants.kRightDriveMasterId, 1,
                DcMotorSimple.Direction.REVERSE, true, true, true);
        bRDrive = new Motor(hardwareMap, Constants.DriveConstants.kRightDriveSlaveId, 1,
                DcMotorSimple.Direction.REVERSE, true, true, true);

        //Intake
        intakeLeft = new Motor(hardwareMap, Constants.IntakeConstants.kIntakeLeftMotorId, 1,
                DcMotorSimple.Direction.REVERSE, false, true, false);
        intakeRight = new Motor(hardwareMap, Constants.IntakeConstants.kIntakeRightMotorId, 1,
                DcMotorSimple.Direction.FORWARD, false, true, false);

        //FlyWheel
        flyWheel = new Motor(hardwareMap, Constants.LiftConstants.kFlyWheel, 1,
                DcMotorSimple.Direction.REVERSE, false, false, true);

        fwArm = hardwareMap.get(Servo.class, Constants.LiftConstants.kFWArm);

        wgGrabber = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGGrabber);
        wgLeft = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGLeft);
        wgLeft.setDirection(Servo.Direction.REVERSE);
        wgRight = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGRight);

        imu = new RevIMU("imu1", hardwareMap);

        // Init all subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.init(auto);
        }
    }

    // Update data from Hubs and Apply new data
    public void update() {
        // Update Bulk Data
        bulkDataMaster = revHubMaster.getBulkInputData();
        bulkDataSlave = revHubSlave.getBulkInputData();

        // Update Motors
        flyWheel.update(bulkDataSlave);

        // Update Subsystems
        for (Subsystem subsystem : subsystems)
            subsystem.update();

    }

    // Output Telemetry for all subsystems
    public void outputToTelemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems)
            subsystem.outputToTelemetry(telemetry);

    }

    // Stop all subsystems
    public void stop(){
        for (Subsystem subsystem : subsystems)
            subsystem.stop();
        revHubMaster.close();
        revHubSlave.close();
    }

    public MecanumDrive getMecanumDrive(){
        return mMecanumDrive;
    }

    public Intake getIntake(){
        return mIntake;
    }

    public FlyWheel getFlyWheel(){
        return mFW;
    }

    public WobbleGoal getWobbleGoal(){
        return mWG;
    }

    public Camera getCamera(){
        return mCamera;
    }

}
