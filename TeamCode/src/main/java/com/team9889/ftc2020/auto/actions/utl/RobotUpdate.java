package com.team9889.ftc2020.auto.actions.utl;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/13/2019.
 */
public class RobotUpdate extends Action {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        Robot.getInstance().update();
        while (timer.milliseconds() < 20) {

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {}
}
