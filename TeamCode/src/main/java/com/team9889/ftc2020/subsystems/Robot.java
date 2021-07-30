package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2020.Constants;
import com.team9889.ftc2020.auto.actions.ActionVariables;
import com.team9889.lib.hardware.Motor;
import com.team9889.lib.hardware.RevIMU;
import com.team9889.lib.roadrunner.drive.RoadRunner;
import com.team9889.lib.roadrunner.drive.StandardTrackingWheelLocalizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
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

    public WebcamName webcam;
    public OpenCvCamera camera;

    public Motor fLDrive, fRDrive, bLDrive, bRDrive;
    public RevIMU imu = null;

    public Motor frontIntake;
    public Motor backIntake;
    public Motor passThrough;
    public Servo leftArm, rightArm;

    public Motor flyWheel;
    public Servo fwArm, fwLock, fwFlap;
    public DistanceSensor hopperDist;

    public Servo wgGrabber, wgLeft, wgRight, autoWG;

    public Servo xCam, yCam;

    public RevBulkData bulkDataMaster, bulkDataSlave;
    public ExpansionHubEx revHubMaster, revHubSlave;

    public HardwareMap hardwareMap;

    public ActionVariables actionVariables = new ActionVariables();

    ElapsedTime robotTimer = new ElapsedTime();

    public double result = Double.POSITIVE_INFINITY;

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

    public RoadRunner rr;
    public StandardTrackingWheelLocalizer localizer;

    public boolean blue = false, blueGoal = false;

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
        webcam = hardwareMap.get(WebcamName.class, Constants.kWebcam);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam);

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
        frontIntake = new Motor(hardwareMap, Constants.IntakeConstants.kFrontIntakeMotorId, 1,
                DcMotorSimple.Direction.FORWARD, false, true, true);
        backIntake = new Motor(hardwareMap, Constants.IntakeConstants.kBackIntakeMotorId, 1,
                DcMotorSimple.Direction.REVERSE, false, true, true);
        passThrough = new Motor(hardwareMap, Constants.IntakeConstants.kPassThroughId, 1,
                DcMotorSimple.Direction.REVERSE, false, true, true);

        leftArm = hardwareMap.get(Servo.class, Constants.IntakeConstants.kLeftArm);
        rightArm = hardwareMap.get(Servo.class, Constants.IntakeConstants.kRightArm);

        //FlyWheel
//        flyWheel = new Motor(hardwareMap, Constants.ShooterConstants.kFlyWheel, 1,
//                DcMotorSimple.Direction.REVERSE, false, false, true);
        flyWheel = new Motor(hardwareMap, Constants.ShooterConstants.kFlyWheel, 1,
                DcMotorSimple.Direction.REVERSE, false, false, true);

        fwArm = hardwareMap.get(Servo.class, Constants.ShooterConstants.kFWArm);
        fwLock = hardwareMap.get(Servo.class, Constants.ShooterConstants.kFWLock);
        fwFlap = hardwareMap.get(Servo.class, Constants.ShooterConstants.kFWFlap);

        hopperDist = hardwareMap.get(DistanceSensor.class, Constants.ShooterConstants.kHopperDist);

        wgGrabber = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGGrabber);
        wgLeft = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGLeft);
        wgLeft.setDirection(Servo.Direction.REVERSE);
        wgRight = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGRight);
        autoWG = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kAutoWG);

        xCam = hardwareMap.get(Servo.class, Constants.CameraConstants.kCameraXId);
        yCam = hardwareMap.get(Servo.class, Constants.CameraConstants.kCameraYId);

        imu = new RevIMU("imu1", hardwareMap);

        // Init all subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.init(auto);
        }

        rr = new RoadRunner(hardwareMap);
        localizer = (StandardTrackingWheelLocalizer) rr.getLocalizer();

        robotTimer.reset();
    }

    // Update data from Hubs and Apply new data
    public void update() {
        // Update Bulk Data
        try {
            bulkDataMaster = revHubMaster.getBulkInputData();
            bulkDataSlave = revHubSlave.getBulkInputData();

            // Update Motors
            flyWheel.update(bulkDataSlave);
            frontIntake.update(bulkDataSlave);
            backIntake.update(bulkDataSlave);
            passThrough.update(bulkDataSlave);

            // Update Subsystems
            for (Subsystem subsystem : subsystems)
                subsystem.update();

            result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0){
                    result = Math.min(result, voltage);
                }
            }

            Robot.getInstance().getIntake().current = passThrough.motor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
        } catch (Exception e){
            Log.v("Exception@robot.update", "" + e);
        }

    }

    // Output Telemetry for all subsystems
    public void outputToTelemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems)
            subsystem.outputToTelemetry(telemetry);

        telemetry.addData("Current", passThrough.motor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
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
