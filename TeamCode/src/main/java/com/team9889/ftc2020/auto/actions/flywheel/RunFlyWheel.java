package com.team9889.ftc2020.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/5/2020.
 */
public class RunFlyWheel extends Action {
    ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        loopTimer.reset();
        timer.reset();
    }

    @Override
    public void update() {
        loopTimer.reset();

        while (loopTimer.milliseconds() < 20) {
        }

        Robot.getInstance().getFlyWheel().setFlyWheelSpeed(5200, loopTimer.milliseconds());
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 8000;
    }

    @Override
    public void done() {

    }
}
