package com.team9889.ftc2020.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 12/5/2020.
 */

public class ShootRings extends Action {
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();
    int stage = 0, ringsShot = 0, rings, time, power = 1145;
    //    1135
    boolean extend = false, stop = false, ps = false;

    Telemetry telemetry;

    public ShootRings (int rings, int time, Telemetry telemetry) {
        this.rings = rings;
        this.time = time;
        this.telemetry = telemetry;
    }

    public ShootRings (int rings, int time, Telemetry telemetry, int power) {
        this.rings = rings;
        this.time = time;
        this.telemetry = telemetry;
        this.power = power;
    }

    public ShootRings (int rings, int time, Telemetry telemetry, int power, boolean stop) {
        this.rings = rings;
        this.time = time;
        this.telemetry = telemetry;
        this.power = power;
        this.stop = stop;
    }

    public ShootRings (int rings, int time, Telemetry telemetry, boolean ps, boolean stop) {
        this.rings = rings;
        this.time = time;
        this.telemetry = telemetry;
        this.ps = ps;
        this.stop = stop;
    }

    @Override
    public void start() {
        shootTimer.reset();
        loopTimer.reset();
        totalTimer.reset();

        if (ps) {
            Robot.getInstance().getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;
        } else {
            Robot.getInstance().fwFlap.setPosition(.4);
        }
        Robot.getInstance().fwLock.setPosition(.4);
        Robot.getInstance().flyWheel.resetEncoder();

        Robot.getInstance().getFlyWheel().time = 200;
    }

    @Override
    public void update() {
//        if (totalTimer.milliseconds() > time && shootTimer.milliseconds() > 150) {
//            if (extend) {
//                Robot.getInstance().fwArm.setPosition(0.47);
//                extend = false;
//                ringsShot++;
//            } else {
//                Robot.getInstance().fwArm.setPosition(.62);
//                extend = true;
//            }
//
//            shootTimer.reset();
//        }

        if (ps) {
            if (Robot.getInstance().getFlyWheel().shootRing(50)) {
                ringsShot++;
            }
        } else {
            if (Robot.getInstance().getFlyWheel().shootRing(100)) {
                ringsShot++;
            }
        }
//        RobotLog.a("Loops Time: " + String.valueOf(loopTimer.milliseconds()) + " | Velocity: " + String.valueOf(Robot.getInstance().flyWheel.getVelocity()));

//        Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;
        telemetry.update();

//        Robot.getInstance().update();
        loopTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return ringsShot >= rings && !extend;
    }

    @Override
    public void done() {
        if (stop) {
            Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.OFF;
        }

        Robot.getInstance().fwLock.setPosition(1);

        Robot.getInstance().getIntake().ringsIntaken = 0;

        Robot.getInstance().getFlyWheel().time = 100;
    }
}
