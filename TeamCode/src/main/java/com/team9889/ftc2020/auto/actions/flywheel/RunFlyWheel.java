package com.team9889.ftc2020.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/5/2020.
 */
public class RunFlyWheel extends Action {
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void update() {
        Robot.getInstance().getFlyWheel().setFlyWheelSpeed(5200, loopTimer.milliseconds());

        while (loopTimer.milliseconds() > 20) {
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
}
