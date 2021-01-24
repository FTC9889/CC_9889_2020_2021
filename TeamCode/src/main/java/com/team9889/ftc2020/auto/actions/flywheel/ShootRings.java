package com.team9889.ftc2020.auto.actions.flywheel;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 12/5/2020.
 */

public class ShootRings extends Action {
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();
    int stage = 0, ringsShot = 0, rings;
    boolean extend = false;

    public ShootRings (int rings) {
        this.rings = rings;
    }

    @Override
    public void start() {
        shootTimer.reset();
        loopTimer.reset();
        totalTimer.reset();

//        Robot.getInstance().getFlyWheel().lastMotorPos = 0;
//        Robot.getInstance().flyWheel.resetEncoder();
    }

    @Override
    public void update() {
        loopTimer.reset();

        if (totalTimer.milliseconds() > 2000 && shootTimer.milliseconds() > 500) {
            if (extend) {
                Robot.getInstance().fwArm.setPosition(.7);
                extend = false;
                ringsShot++;
            } else {
                Robot.getInstance().fwArm.setPosition(0);
                extend = true;
            }

            shootTimer.reset();
        }

        while (loopTimer.milliseconds() < 20){}

        RobotLog.a("Loops Time: " + String.valueOf(loopTimer.milliseconds()) + " | Velocity: " + String.valueOf(Robot.getInstance().flyWheel.getVelocity()));

        Robot.getInstance().getFlyWheel().setFlyWheelSpeed(5200, loopTimer.milliseconds());
    }

    @Override
    public boolean isFinished() {
        return ringsShot >= rings;
    }

    @Override
    public void done() {
        Robot.getInstance().flyWheel.setPower(0);
        Log.i("Shot", "");
    }
}
