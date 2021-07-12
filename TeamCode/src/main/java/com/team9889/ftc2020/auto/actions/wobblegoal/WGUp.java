package com.team9889.ftc2020.auto.actions.wobblegoal;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 1/23/2021.
 */
public class WGUp extends Action {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        timer.reset();
        Robot.getInstance().getWobbleGoal().pickUpWG();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 500;
    }

    @Override
    public void done() {

    }
}
