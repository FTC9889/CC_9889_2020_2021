package com.team9889.ftc2020.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 6/7/2021.
 */
class PowerShots extends Action {
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();
    int stage = 0, ringsShot = 0, rings, time, power = 1145;
    //    1135
    boolean extend = false, stop = true;

    @Override
    public void start() {
        shootTimer.reset();
        loopTimer.reset();
        totalTimer.reset();

        Robot.getInstance().flyWheel.motor.setVelocity(power);

        Robot.getInstance().fwFlap.setPosition(.55);
        Robot.getInstance().fwLock.setPosition(.4);
    }

    @Override
    public void update() {
        Robot.getInstance().passThrough.setPower(.4);




        Robot.getInstance().update();
        loopTimer.reset();
    }

    public static double calculateAngle() {
        double x = Robot.getInstance().rr.getLocalizer().getPoseEstimate().getX() + 63;
        double y = Robot.getInstance().rr.getLocalizer().getPoseEstimate().getY() + 63;

        double hypot = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

//        double hypot = Math.sqrt(Math.pow(-20, 2) + Math.pow(-30, 2));

        double angle = Robot.getInstance().rr.getLocalizer().getPoseEstimate().getY() / hypot;
        angle = Math.sin(angle);

        return angle;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
}
