package com.team9889.ftc2020.auto.actions.flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.control.controllers.PID;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShotsAuto extends Action {
    public static double p = .5, d = 30;

    private PID camOrientationPID = new PID(0.5, 0, 30);

    @Override
    public void start() {}

    @Override
    public void update() {
        updatePIDValues();

        double cameraTarget = Robot.getInstance().getCamera().getPosOfTarget().x;

        double speed = 0;
        if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) >= 0.05){
            speed = -camOrientationPID.update(cameraTarget, 0);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed);
    }

    void updatePIDValues () {
        camOrientationPID.p = p;
        camOrientationPID.d = d;
    }
    @Override
    public boolean isFinished() {
//        return num >= 4;
        return false;
    }

    @Override
    public void done() {
        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.OFF);
        Robot.getInstance().getFlyWheel().psPower = false;
        Robot.getInstance().fwLock.setPosition(1);
    }
}
