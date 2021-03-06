package com.team9889.ftc2020.subsystems;

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
            Robot.getInstance().wgGrabber.setPosition(.7);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {
        if (currentArmPos != wantedArmPos) {
            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentArmPos = wantedArmPos;
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }

            switch (wantedArmPos) {
                case UP:
                    Robot.getInstance().wgLeft.setPosition(.44);
                    Robot.getInstance().wgRight.setPosition(.44);
                    break;

                case DOWN:
                    Robot.getInstance().wgLeft.setPosition(.8);
                    Robot.getInstance().wgRight.setPosition(.8);
                    break;

                case IN:
                    Robot.getInstance().wgLeft.setPosition(0.4);
                    Robot.getInstance().wgRight.setPosition(0.4);
                    break;

                case NULL:
                    break;
            }

            currentArmPos = wantedArmPos;
        }

        if (wantedGrabberOpen) {
            Robot.getInstance().wgGrabber.setPosition(.7);

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentGrabberOpen = true;
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }
        } else {
            Robot.getInstance().wgGrabber.setPosition(.25);

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 500) {
                currentGrabberOpen = false;
            }

            if (Robot.getInstance().robotTimer.milliseconds() - grabberTime > 700) {
                grabberTime = Robot.getInstance().robotTimer.milliseconds();
            }
        }
    }

    @Override
    public void stop() {

    }

    public void putWGDown() {
        if (currentArmPos != wgArmPositions.DOWN) {
            wantedArmPos = wgArmPositions.DOWN;
        } else {
            wantedGrabberOpen = true;
        }
    }

    public void pickUpWG() {
        if (currentGrabberOpen) {
            wantedGrabberOpen = false;
        } else {
            wantedArmPos = wgArmPositions.UP;
        }
    }
}
