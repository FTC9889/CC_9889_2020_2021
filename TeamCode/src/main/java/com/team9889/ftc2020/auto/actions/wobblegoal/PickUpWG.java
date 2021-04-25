package com.team9889.ftc2020.auto.actions.wobblegoal;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 1/23/2021.
 */
public class PickUpWG extends Action {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (timer.milliseconds() < 700) {
            Robot.getInstance().wgGrabber.setPosition(0.25);
        } else {
            Robot.getInstance().wgLeft.setPosition(.4);
            Robot.getInstance().wgRight.setPosition(.4);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 1000;
    }

    @Override
    public void done() {

    }
}
