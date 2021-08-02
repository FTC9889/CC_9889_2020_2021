package com.team9889.ftc2020;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

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

    // Dashboard
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public int timeToWait = 0;
    boolean buttonReleased = true;

    public void waitForStart(boolean autonomous) {
        this.waitForStart(autonomous, AutoModeBase.StartPosition.REDRIGHT);
    }

    public void waitForStart(boolean autonomous, AutoModeBase.StartPosition startPosition) {
        Robot.init(hardwareMap, autonomous);
        Robot.update();

        if (Constants.pose != null) {
            if (Constants.pose.equals(new Pose2d(0, 0, 0))) {
                Robot.rr.getLocalizer().setPoseEstimate(new Pose2d(-63, -17.5, Math.toRadians(0)));
            } else {
                Robot.rr.getLocalizer().setPoseEstimate(Constants.pose);
            }
        }

        if (autonomous) {
            if (startPosition == AutoModeBase.StartPosition.REDRIGHT ||
                startPosition == AutoModeBase.StartPosition.BLUERIGHT) {
                Robot.getCamera().setRSCamPosLeft();
            } else if (startPosition == AutoModeBase.StartPosition.REDLEFT ||
                startPosition == AutoModeBase.StartPosition.BLUELEFT) {
                Robot.getCamera().setRSCamPosRight();
            }

            if (startPosition == AutoModeBase.StartPosition.REDLEFT ||
                startPosition == AutoModeBase.StartPosition.REDRIGHT) {
                Constants.side = Color.RED;
            } else if (startPosition == AutoModeBase.StartPosition.BLUELEFT ||
                    startPosition == AutoModeBase.StartPosition.BLUERIGHT) {
                Constants.side = Color.BLUE;
            }

            Robot.getCamera().update();
        }

        telemetry.setMsTransmissionInterval(autonomous ? 50:1000);

//        telemetry = dashboard.getTelemetry();

        if(autonomous){
            // Autonomous Init Loop code
            while(isInInitLoop()){
                telemetry.addData("Waiting for Start","");
                telemetry.addData("Box", Robot.getCamera().getRSBox().toString());

                telemetry.addData("Delay at beginning", timeToWait / 1000);

                Robot.outputToTelemetry(telemetry);
                telemetry.update();

                FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);

                if (gamepad1.dpad_up && buttonReleased) {
                    timeToWait += 1000;
                    buttonReleased = false;
                } else if (gamepad1.dpad_down && buttonReleased) {
                    timeToWait -= 1000;
                    buttonReleased = false;
                } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                    buttonReleased = true;
                }
            }
        } else {
            Robot.blue = Constants.side == Color.BLUE;

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
        Constants.pose = Robot.rr.getPoseEstimate();
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
