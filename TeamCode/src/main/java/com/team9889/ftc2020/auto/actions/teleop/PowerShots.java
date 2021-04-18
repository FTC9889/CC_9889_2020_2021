package com.team9889.ftc2020.auto.actions.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShots extends Action {
    public static double p = .75, i = 0, d = 10, max = 0;

    private PIDF camOrientationPID = new PIDF(1, 0, 8, 100);

    boolean first = true;
    int num = 1, ready = 0;
    ElapsedTime timer = new ElapsedTime(), beginning = new ElapsedTime();

    @Override
    public void start() {
        Robot.getInstance().getCamera().setPS1CamPos();
        Robot.getInstance().getCamera().setScanForGoal();
        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);
        Robot.getInstance().getFlyWheel().psPower = true;
        Robot.getInstance().fwLock.setPosition(.4);

        beginning.reset();
    }

    @Override
    public void update() {
//        Robot.getInstance().update();

        if (first && beginning.milliseconds() > 600) {
            if (ready < 5) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    turn(1);
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().getCamera().setPS2CamPos();
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 2;
                    ready = 0;
                    first = false;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (num == 2 && !first) {
            if (ready < 6) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    turn(1.2);
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().getCamera().setPS3CamPos();
                    Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT3);
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 3;
                    ready = 0;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (!first) {
            if (ready < 6) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    turn(1.1);
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 4;
                    ready = 0;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        }
    }

    void turn (double speedMultiplier) {
        camOrientationPID.p = p;
        camOrientationPID.i = i;
        camOrientationPID.d = d;
        camOrientationPID.maxIntegral = max;

        double turn = Robot.getInstance().getMecanumDrive().getAngle().getTheda(AngleUnit.DEGREES) -
                Math.toDegrees(Robot.getInstance().getMecanumDrive().angleFromAuton);

        double speed = 0;
        if (Robot.getInstance().getCamera().getPosOfTarget().x != 1e10) {
            double camera = Robot.getInstance().getCamera().getPosOfTarget().x;
            camOrientationPID.update(camera, 0);
            speed = -CruiseLib.limitValue(camOrientationPID.getOutput() / 1.2, -.12, -.6, .12, .6);
        } else if (Math.abs(turn) > 25) {
            camOrientationPID.update(turn, 20);
            speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.6, .05, .6);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed * speedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return num >= 4;
    }

    @Override
    public void done() {
//        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.DEFAULT);
        Robot.getInstance().getFlyWheel().psPower = false;
        Robot.getInstance().fwLock.setPosition(1);
    }
}
