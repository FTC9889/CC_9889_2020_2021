package com.team9889.ftc2019.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.team9889.ftc2019.auto.actions.Action;
import com.team9889.ftc2019.subsystems.Robot;

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
    public void setup(String args) {

    }

    @Override
    public void start() {
        shootTimer.reset();
        loopTimer.reset();
        totalTimer.reset();
    }

    @Override
    public void update() {
        loopTimer.reset();

//        if (totalTimer.milliseconds() > 2000 && shootTimer.milliseconds() > 500) {
//            if (extend) {
//                Robot.getInstance().fwArm.setPosition(1);
//                extend = false;
//                ringsShot++;
//            } else {
//                Robot.getInstance().fwArm.setPosition(0);
//                extend = true;
//            }
//
//            shootTimer.reset();
//        }
        while (loopTimer.milliseconds() < 20){}

        RobotLog.a("Loops Time: " + String.valueOf(loopTimer.milliseconds()) + " | Velocity: " + String.valueOf(Robot.getInstance().flyWheel.getVelocity()));

        Robot.getInstance().getFlyWheel().setFlyWheelSpeed(1000, loopTimer.milliseconds());
    }

    @Override
    public boolean isFinished() {
        return ringsShot >= rings;
    }

    @Override
    public void done() {

    }
}
