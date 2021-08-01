package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class WobbleGoal extends Subsystem{
    public enum wgArmPositions {
        UP, DOWN, IN, NULL
    }
    public wgArmPositions currentArmPos = wgArmPositions.NULL;
    public wgArmPositions wantedArmPos = wgArmPositions.NULL;

    double grabberTime = 0, armTime = 0;

    public boolean wantedGrabberOpen = true, currentGrabberOpen = true;

    @Override
    public void init(boolean auto) {
        if (auto) {
            Robot.getInstance().wgGrabber.setPosition(.9);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {
//        if (currentArmPos != wantedArmPos) {
            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentArmPos = wantedArmPos;
            }

            switch (wantedArmPos) {
                case UP:
                    Robot.getInstance().wgLeft.setPosition(.44);
                    Robot.getInstance().wgRight.setPosition(.44);
                    break;

                case DOWN:
//                    Robot.getInstance().wgLeft.setPosition(.8);
//                    Robot.getInstance().wgRight.setPosition(.8);
                    Robot.getInstance().wgLeft.setPosition(.9);
                    Robot.getInstance().wgRight.setPosition(.9);
                    break;

                case IN:
                    Robot.getInstance().wgLeft.setPosition(0.4);
                    Robot.getInstance().wgRight.setPosition(0.4);
                    break;

                case NULL:
                    break;
            }

//            currentArmPos = wantedArmPos;
//        }

        if (wantedGrabberOpen) {
            Robot.getInstance().wgGrabber.setPosition(.7);

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentGrabberOpen = true;
            }
        } else {
            Robot.getInstance().wgGrabber.setPosition(.25);

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentGrabberOpen = false;
            }
        }
    }

    @Override
    public void stop() {

    }

    public ElapsedTime wgTimer = new ElapsedTime();
    public void putWGDown() {

//        if (currentArmPos != wgArmPositions.DOWN) {
        if (wgTimer.milliseconds() < 500)
            wantedArmPos = wgArmPositions.DOWN;
//        } else {
        else {
            wantedGrabberOpen = true;
        }

        wantedArmPos = wgArmPositions.DOWN;
//        }
    }

    public void pickUpWG() {
//        if (currentGrabberOpen) {
        if (wgTimer.milliseconds() < 500)
            wantedGrabberOpen = false;
//        } else {
        else {
//            wantedGrabberOpen = false;
            wantedArmPos = wgArmPositions.UP;
            Log.i("PickUp", "");
        }
//        }
    }
}
