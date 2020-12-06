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


/**
 * Created by Eric on 7/26/2019.
 */

public class Robot{

//    public WebcamName webcam;

    public Motor fLDrive, fRDrive, bLDrive, bRDrive;
    public RevIMU imu = null;

    public Motor intakeLeft, intakeRight;

    public Motor flyWheel;
    public Servo fwArm;

    public Servo wgGrabber, wgLeft, wgRight;

    public boolean redAuto;

    public RevBulkData bulkDataMaster, bulkDataSlave;
    ExpansionHubEx revHubMaster, revHubSlave;

    public HardwareMap hardwareMap;

    public static ElapsedTime timer = new ElapsedTime();
    public static double lastTime = 0;
    public static ElapsedTime gyroTimer = new ElapsedTime();

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

    private boolean mAuto = false;

    boolean debugging = false;
//    private com.team9889.lib.android.FileWriter writer = new FileWriter("Drive3.csv");
//    private com.team9889.lib.android.FileWriter poseWriter = new FileWriter("Pose.csv");

    public void init(HardwareMap hardwareMap, boolean auto){
        timer.reset();
        gyroTimer.reset();
        this.mAuto = auto;
        this.hardwareMap = hardwareMap;

        Date currentData = new Date();
        SimpleDateFormat format = new SimpleDateFormat("dd.M.yyyy hh:mm:ss");

        RobotLog.a("Robot Init Started at " + format.format(currentData));

        // Rev Hubs
        revHubMaster = hardwareMap.get(ExpansionHubEx.class, Constants.kRevHubMaster);
        revHubSlave = hardwareMap.get(ExpansionHubEx.class, Constants.kRevHubSlave);

        // Camera
//        webcam = hardwareMap.get(WebcamName.class, Constants.kWebcam);

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
        PIDCoefficients pidNew = new PIDCoefficients(.008, 0, 0);
        flyWheel = new Motor(hardwareMap, Constants.LiftConstants.kFlyWheel, 1,
                DcMotorSimple.Direction.REVERSE, false, false, true);

        fwArm = hardwareMap.get(Servo.class, Constants.LiftConstants.kFWArm);

        wgGrabber = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGGrabber);
        wgLeft = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGLeft);
        wgLeft.setDirection(Servo.Direction.REVERSE);
        wgRight = hardwareMap.get(Servo.class, Constants.WobbleGoalConstants.kWGRight);

        imu = new RevIMU("imu1", hardwareMap);

        if (auto)
            debugging = true;

//        if(debugging) writer.write("clock,x,y,theda");

        getMecanumDrive().init(auto);
        getIntake().init(auto);
        getFlyWheel().init(auto);
        getWobbleGoal().init(auto);
        getCamera().init(auto);

        timer.reset();
    }


    private ElapsedTime updateTimer = new ElapsedTime();
    public void update(){
        RobotLog.v("Loop Time: " + String.valueOf(timer.milliseconds()) + " | dt: " + String.valueOf(updateTimer.milliseconds()));

        getMecanumDrive().getAngle().getTheda(AngleUnit.RADIANS);

        if (false)
            bulkDataMaster = revHubMaster.getBulkInputData();

        bulkDataSlave = revHubSlave.getBulkInputData();
        RobotLog.a("Volts: " + String.valueOf(revHubSlave.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)));

        flyWheel.update(bulkDataSlave);

        mMecanumDrive.update();

//        if(debugging)
//            writer.write(updateTimer.milliseconds() + "," + getMecanumDrive().getCurrentPose().getX() + ","
//                    + getMecanumDrive().getCurrentPose().getY() + ","
//                    + -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.RADIANS)
//            );

        updateTimer.reset();

        while (timer.milliseconds() - lastTime < 25){}

        lastTime = (int) timer.milliseconds();
    }

    public void outputToTelemetry(Telemetry telemetry) {
        getMecanumDrive().outputToTelemetry(telemetry);
        getFlyWheel().outputToTelemetry(telemetry);

        if(false) {
            telemetry.addData("Loop Time", (timer.milliseconds() - lastTime));
        }
    }

    public void stop(){
        for (Motor motor:Arrays.asList(fLDrive, fRDrive, bLDrive, bRDrive, intakeLeft, intakeRight)) {
            motor.setPower(0);
        }
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
