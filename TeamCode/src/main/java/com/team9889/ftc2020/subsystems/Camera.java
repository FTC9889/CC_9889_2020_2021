package com.team9889.ftc2020.subsystems;

import android.graphics.RectF;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.lib.detectors.ScanForGoalTFL;
import com.team9889.lib.detectors.ScanForRS;
import com.team9889.lib.detectors.ScanForWG;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Created by MannoMation on 10/27/2018.
 */

@Config
public class Camera extends Subsystem{
    public static double tPS1 = .53, tPS2 = .54, tPS3 = .58, tPS4 = .59;
    public static double bluetPS1 = .42, bluetPS2 = .4, bluetPS3 = .39, bluetPS4 = .36;

    public double camYPose = .7;

//    public ScanForGoal scanForGoal = new ScanForGoal();
    ScanForWG scanForWG = new ScanForWG();
    ScanForRS scanForRS = new ScanForRS();
    public ScanForGoalTFL scanForGoal;

    ElapsedTime timer = new ElapsedTime();

    public enum CameraStates {
        GOAL, PS1, PS2, PS3, RSRIGHT, RSLEFT, NULL
    }
    public CameraStates currentCamState = CameraStates.NULL;
    public CameraStates wantedCamState = CameraStates.NULL;

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
                }
            }
        });

        scanForGoal = new ScanForGoalTFL();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Pos Of Target", getPosOfTarget());
        telemetry.addData("Camera Error", Math.abs(Math.round(getPosOfTarget().x * 100.0))/100.0);
    }

    @Override
    public void update() {
        if (currentCamState != wantedCamState) {
            switch (wantedCamState) {
                case GOAL:
                    setCamPositions(.485, .7);
                    break;

                case PS1:
                    setCamPositions(.52, .7);
                    break;

                case PS2:
                    setCamPositions(.54, .7);
                    break;

                case PS3:
                    setCamPositions(.57, .7);
                    break;

                case RSRIGHT:
                    setCamPositions(.64, .9);
                    break;

                case RSLEFT:
                    setCamPositions(.4, .9);
                    break;

                case NULL:
                    break;
            }

            currentCamState = wantedCamState;
        }
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

    public void setGoalCamPos () {
        wantedCamState = CameraStates.GOAL;
    }

    public void setPS1CamPos () {
        wantedCamState = CameraStates.PS1;
    }

    public void setPS2CamPos () {
        wantedCamState = CameraStates.PS2;
    }

    public void setPS3CamPos () {
        wantedCamState = CameraStates.PS3;
    }

    public void setRSCamPosRight() {
        wantedCamState = CameraStates.RSRIGHT;
    }

    public void setRSCamPosLeft() {
        wantedCamState = CameraStates.RSLEFT;
    }


    public void setTelePS1CamPos () {
        wantedCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS1);
        } else {
            Robot.getInstance().xCam.setPosition(tPS1);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS2CamPos () {
        wantedCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS2);
        } else {
            Robot.getInstance().xCam.setPosition(tPS2);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS3CamPos () {
        wantedCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS3);
        } else {
            Robot.getInstance().xCam.setPosition(tPS3);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }

    public void setTelePS4CamPos () {
        wantedCamState = CameraStates.GOAL;
        if (Robot.getInstance().blue) {
            Robot.getInstance().xCam.setPosition(bluetPS4);
        } else {
            Robot.getInstance().xCam.setPosition(tPS4);
        }
        Robot.getInstance().yCam.setPosition(.7);
    }


    public void setCamPositions(double x, double y) {
        Robot.getInstance().xCam.setPosition(x);
        Robot.getInstance().yCam.setPosition(y);
    }

    public static double widthOfGoal = 23.875, f = 426.89;
    public Pose2d getRobotPos() {
        ScanForGoalTFL scan = Robot.getInstance().getCamera().scanForGoal;
        if (scan.goal != null) {
            RectF bb = scan.goal.getBoundingBox();
            if (scan.goal.getCategories().get(0).getScore() > 0.9 && timer.milliseconds() > 0
                    && bb.left > 5 && bb.right < 315 && bb.top < 235 && bb.bottom > 5) {
                float p = bb.right - bb.left;
                double dist = ((widthOfGoal * f) / p);
                Log.i("Goal Position", "" + dist);

                double goalPos = ((bb.centerX()) / 160) - 1;
                double fovAngle = goalPos * (53.4 / 2) * 0.625;
                double fullAngle = fovAngle + Math.toDegrees(scan.odoPos.getHeading());
                Log.i("Cam to Goal Angle", "" + fullAngle);

                Pose2d pos = new Pose2d(-Math.cos(Math.toRadians(fullAngle)) * dist,
                        Math.sin(Math.toRadians(fullAngle)) * dist,
                        -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.RADIANS));

                Log.i("Goal Pos", "" + pos);
                pos = pos.plus(new Pose2d(64, -36, 0));

//                pos = new Pose2d(pos.getX(), pos.getY(), Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES));

                Log.i("Robot Pos", "" + pos);

                timer.reset();
                return pos;
            }
        }

        return null;
    }
}
