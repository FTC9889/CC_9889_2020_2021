package com.team9889.ftc2020;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.detectors.ScanForSkyStonesPipeline;
import com.team9889.lib.detectors.TeleOpStonePipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by joshua9889 on 3/28/2018.
 *
 * This class extends LinearOpMode and makes it
 * easier to make code for the robot and not copy
 * and pasting init code.
 */

public abstract class Team9889Linear extends LinearOpMode {

    // Robot Object
    protected Robot Robot = com.team9889.ftc2020.subsystems.Robot.getInstance();

    // Match Timer
    protected ElapsedTime matchTime = new ElapsedTime();

    // Background
//    private View relativeLayout;

//    OpenCvCamera phoneCam;
//    public double positionOfSkyStone;
//    TeleOpStonePipeline pipeline = new TeleOpStonePipeline();

    public void waitForStart(boolean autonomous) {
//        int relativeLayoutId = hardwareMap.appContext.getResources().
//                getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        Robot.init(hardwareMap, autonomous);

        telemetry.setMsTransmissionInterval(autonomous ? 50:1000);

        if(autonomous){
//            phoneCam.openCameraDevice();
//            ScanForSkyStonesPipeline scanForSkyStonesPipeline = new ScanForSkyStonesPipeline();
//            phoneCam.setPipeline(scanForSkyStonesPipeline);
//            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);


            // Autonomous Init Loop code
            while(isInInitLoop()){
                telemetry.addData("Waiting for Start","");
//                positionOfSkyStone = scanForSkyStonesPipeline.getPositionOfSkyStone();
//                telemetry.addData("Position", scanForSkyStonesPipeline.getPositionOfSkyStone());
                Robot.outputToTelemetry(telemetry);
                telemetry.update();
            }
//            Runnable ShutDownCameraThread = new Runnable() {
//                @Override
//                public void run() {
//                    phoneCam.stopStreaming();
//                }
//            };

//            new Thread(ShutDownCameraThread).start();
        } else {
//            phoneCam.openCameraDevice();
//            phoneCam.setPipeline(pipeline);
//            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

            // Teleop Init Loop code
            while(isInInitLoop()){
                telemetry.addData("Waiting for Start","");
                Robot.getMecanumDrive().outputToTelemetry(telemetry);
                telemetry.update();
            }
        }

        matchTime.reset();
    }

    /**
     * Used to stop everything (Robot and OpMode).
     */
    protected void finalAction(){
        Robot.stop();
        requestOpModeStop();
    }

    /**
     * @return Is the robot waiting for start
     */
    private synchronized boolean isInInitLoop(){
        return !isStarted() && !isStopRequested();
    }

//    ArrayList<Action> actions = new ArrayList<>();
//    public void runAction(Action action){
//        if(opModeIsActive())
//            action.start();
//
//        actions.add(action);
//    }
//
//    public void UpdateActions() {
//        if (actions.size() > 0) {
//            for (int i = 0; i < actions.size(); i++) {
//                actions.get(i).update();
//
//                if (actions.get(i).isFinished()) {
//                    actions.get(i).done();
//                    actions.remove(actions.get(i));
//                }
//            }
//        }
//    }

    public void runAction(Action action){
        if(opModeIsActive() && !isStopRequested())
            action.start();

        while (!action.isFinished() && opModeIsActive() && !isStopRequested()) {
            action.update();
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

    // TODO: Convert all actions to use serial/parallel actions?
    public void ThreadAction(final Action action){
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                runAction(action);
            }
        };

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }
}
