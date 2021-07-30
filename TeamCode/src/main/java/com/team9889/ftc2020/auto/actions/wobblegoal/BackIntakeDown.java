package com.team9889.ftc2020.auto.actions.wobblegoal;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 7/29/2021.
 */
public class BackIntakeDown extends Action {
    ElapsedTime timer = new ElapsedTime();
    double time = 0;
    boolean first = true;

    public BackIntakeDown() {

    }

    @Override
    public void start() {
        Robot.getInstance().getWobbleGoal().wgTimer.reset();
        timer.reset();
    }

    @Override
    public void update() {
        if (timer.milliseconds() < 1000) {
            Robot.getInstance().getWobbleGoal().putWGDown();
        } else {
            if (first) {
                Robot.getInstance().getWobbleGoal().wgTimer.reset();
                first = false;
            } else {
                Robot.getInstance().getWobbleGoal().pickUpWG();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 2000;
    }

    @Override
    public void done() {

    }
}
