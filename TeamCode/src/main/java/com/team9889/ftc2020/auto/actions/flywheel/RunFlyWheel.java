package com.team9889.ftc2020.auto.actions.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/5/2020.
 */

public class RunFlyWheel extends Action {
    ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    int time = 0;

    public RunFlyWheel (int time) {
        this.time = time;
    }

    @Override
    public void start() {
        loopTimer.reset();
        timer.reset();
    }

    @Override
    public void update() {

        while (loopTimer.milliseconds() < 20) {
        }

//        Robot.getInstance().getFlyWheel().setFlyWheelSpeed(3100, loopTimer.milliseconds());

//        Robot.getInstance().update();
        loopTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > time;
    }

    @Override
    public void done() {

    }
}
