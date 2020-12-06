package com.team9889.ftc2020.auto.actions.utl;

import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/13/2019.
 */
public class RobotUpdate extends Action {
    @Override
    public void start() {

    }

    @Override
    public void update() {
        Robot.getInstance().update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {}
}
