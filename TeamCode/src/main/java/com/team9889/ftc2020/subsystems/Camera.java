package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.detectors.ScanForGoal;
import com.team9889.lib.detectors.ScanForRS;
import com.team9889.lib.detectors.ScanForWG;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Created by MannoMation on 10/27/2018.
 */

@Config
public class Camera extends Subsystem{
    public static double p = -0.016, i, d;
    PID cameraY = new PID(-0.016, 0, 0);
    public double camYPose = .7;

    public ScanForGoal scanForGoal = new ScanForGoal();
    ScanForWG scanForWG = new ScanForWG();
    ScanForRS scanForRS = new ScanForRS();

    public enum CameraStates {
        GOAL, WG, RS, NULL
    }
    public CameraStates currentCamState = CameraStates.NULL;

    enum Pipelines {
        GOAL, WG, RS, NULL
    }
    public Pipelines currentPipeline = Pipelines.NULL;

    @Override
    public void init(final boolean auto) {
        Robot.getInstance().camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Robot.getInstance().camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                if (auto) {
                    setScanForRS();
                    setRSCamPos();
                } else {
                    setScanForGoal();
                    Log.i("Hi", "");
//                    setGoalCamPos();
                }
            }
        });
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Pos Of Target", getPosOfTarget());
        telemetry.addData("Box", getRSBox().toString());
    }

    @Override
    public void update() {
        cameraY.p = p;
        cameraY.i = i;
        cameraY.d = d;
    }

    @Override
    public void stop() {
        Robot.getInstance().camera.stopStreaming();
    }

    public AutoModeBase.Boxes getRSBox () {
        AutoModeBase.Boxes box = AutoModeBase.Boxes.CLOSE;
        if (Math.abs(getPosOfTarget().y) == 0) {
            box = AutoModeBase.Boxes.CLOSE;
        } else if (Math.abs(getPosOfTarget().y) < .16) {
            box = AutoModeBase.Boxes.MIDDLE;
        } else if (Math.abs(getPosOfTarget().y) >= .16) {
            box = AutoModeBase.Boxes.FAR;
        }

        return box;
    }

    public void setScanForGoal () {
        Robot.getInstance().camera.setPipeline(scanForGoal);
        currentPipeline = Pipelines.GOAL;
    }

    public void setScanForWG () {
        Robot.getInstance().camera.setPipeline(scanForWG);
        currentPipeline = Pipelines.WG;
    }

    public void setScanForRS () {
        Robot.getInstance().camera.setPipeline(scanForRS);
        currentPipeline = Pipelines.RS;
    }

    public Point getPosOfTarget () {
        Point posToReturn = new Point();
        switch (currentPipeline) {
            case GOAL:
                posToReturn = scanForGoal.getPoint();
                break;

            case WG:
                posToReturn = scanForWG.getPoint();
                break;
            case RS:
                posToReturn = scanForRS.getPoint();
                break;
        }

        return posToReturn;
    }

    public void setWGCamPos () {
        currentCamState = CameraStates.WG;
        Robot.getInstance().xCam.setPosition(1);
        Robot.getInstance().yCam.setPosition(.85);
    }

    public void setGoalCamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.285);

//        camYPose += cameraY.update(getPosOfTarget().y, 0);
//        CruiseLib.limitValue(camYPose, 1, 0);
//        Robot.getInstance().yCam.setPosition(camYPose);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS1CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.32);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS2CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.34);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS3CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.37);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setRSCamPos () {
        currentCamState = CameraStates.RS;
        Robot.getInstance().xCam.setPosition(.37);
        Robot.getInstance().yCam.setPosition(.895);
    }
}
