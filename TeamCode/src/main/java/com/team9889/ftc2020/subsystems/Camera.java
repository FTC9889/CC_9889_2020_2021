package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.detectors.ScanForGoal;
import com.team9889.lib.detectors.ScanForRS;
import com.team9889.lib.detectors.ScanForWG;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Created by MannoMation on 10/27/2018.
 */

@Config
public class Camera extends Subsystem{
    public static double ps1 = .52, ps2 = .54, ps3 = .57;
    public static double tPS1 = .53, tPS2 = .54, tPS3 = .58, tPS4 = .59;
    public static double bluetPS1 = .42, bluetPS2 = .4, bluetPS3 = .39, bluetPS4 = .36;

    public double p = -0.016, i, d;
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
        telemetry.addData("Camera Error", Math.abs(Math.round(getPosOfTarget().x * 100.0))/100.0);
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
        } else if (Math.abs(getPosOfTarget().y) < 17) {
            box = AutoModeBase.Boxes.MIDDLE;
        } else if (Math.abs(getPosOfTarget().y) >= 17) {
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
        Robot.getInstance().xCam.setPosition(.465);

//        camYPose += cameraY.update(getPosOfTarget().y, 0);
//        CruiseLib.limitValue(camYPose, 1, 0);
//        Robot.getInstance().yCam.setPosition(camYPose);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS1CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(ps1);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS2CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(ps2);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS3CamPos () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(ps3);
        Robot.getInstance().yCam.setPosition(.7);
    }


    public void setTelePS1CamPos () {
        currentCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS1);
        } else {
            Robot.getInstance().xCam.setPosition(tPS1);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS2CamPos () {
        currentCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS2);
        } else {
            Robot.getInstance().xCam.setPosition(tPS2);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS3CamPos () {
        currentCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS3);
        } else {
            Robot.getInstance().xCam.setPosition(tPS3);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS4CamPos () {
        currentCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS4);
        } else {
            Robot.getInstance().xCam.setPosition(tPS4);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS1CamPosAuto () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.534);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS2CamPosAuto () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.535);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setPS3CamPosAuto () {
        currentCamState = CameraStates.GOAL;
        Robot.getInstance().xCam.setPosition(.548);
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setRSCamPosRight() {
        currentCamState = CameraStates.RS;
        Robot.getInstance().xCam.setPosition(.64);
        Robot.getInstance().yCam.setPosition(.9);
    }

    public void setRSCamPosLeft() {
        currentCamState = CameraStates.RS;
        Robot.getInstance().xCam.setPosition(.4);
        Robot.getInstance().yCam.setPosition(.9);
    }
}
