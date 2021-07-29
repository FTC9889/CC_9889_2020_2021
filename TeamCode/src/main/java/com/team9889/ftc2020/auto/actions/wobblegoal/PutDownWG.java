package com.team9889.ftc2020.auto.actions.wobblegoal;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 1/23/2021.
 */
public class PutDownWG extends Action {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        Robot.getInstance().getWobbleGoal().wgTimer.reset();
        timer.reset();
    }

    @Override
    public void update() {
        Robot.getInstance().getWobbleGoal().putWGDown();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 1000;
    }

    @Override
    public void done() {

    }
}
