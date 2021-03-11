package com.team9889.ftc2020.auto.actions.flywheel;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 12/5/2020.
 */

public class ShootRings extends Action {
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();
    int stage = 0, ringsShot = 0, rings, time;
    boolean extend = false;

    Telemetry telemetry;

    public ShootRings (int rings, int time, Telemetry telemetry) {
        this.rings = rings;
        this.time = time;
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        shootTimer.reset();
        loopTimer.reset();
        totalTimer.reset();

        Robot.getInstance().getFlyWheel().lastMotorPos = 0;
        Robot.getInstance().flyWheel.resetEncoder();
    }

    @Override
    public void update() {
        if (totalTimer.milliseconds() > time && shootTimer.milliseconds() > 500) {
            if (extend) {
                Robot.getInstance().flyWheelHammer.setPosition(0.45);
                extend = false;
                ringsShot++;
            } else {
                Robot.getInstance().flyWheelHammer.setPosition(1);
                extend = true;
            }

            shootTimer.reset();
        }

        while (loopTimer.milliseconds() < 20){}

//        RobotLog.a("Loops Time: " + String.valueOf(loopTimer.milliseconds()) + " | Velocity: " + String.valueOf(Robot.getInstance().flyWheel.getVelocity()));

        Robot.getInstance().flyWheel.motor.setVelocity(1150);

//        2950
        telemetry.addData("Speed", Robot.getInstance().getFlyWheel().flySpeed);
        telemetry.update();

        Robot.getInstance().update();
        loopTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return ringsShot >= rings && !extend;
    }

    @Override
    public void done() {
        Robot.getInstance().flyWheel.setPower(0);
        Log.i("Shot", "");
    }
}
