package com.team9889.ftc2020.auto.actions.wobblegoal;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 1/23/2021.
 */
public class PickUpWG extends Action {
    ElapsedTime timer = new ElapsedTime();
    double time = 0;

    public PickUpWG() {

    }

    public PickUpWG(double time) {
        this.time = time;
    }

    @Override
    public void start() {
        Robot.getInstance().getWobbleGoal().wgTimer.reset();
        timer.reset();
    }

    @Override
    public void update() {
        Robot.getInstance().getWobbleGoal().pickUpWG();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 1000 + time;
    }

    @Override
    public void done() {

    }
}
