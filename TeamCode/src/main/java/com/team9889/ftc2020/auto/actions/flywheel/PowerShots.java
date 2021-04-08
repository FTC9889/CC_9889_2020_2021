package com.team9889.ftc2020.auto.actions.flywheel;

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
public class PowerShots extends Action {
    public static double p = 1, i = 0, d = 8, f = 50;

    private PIDF camOrientationPIDF = new PIDF(1, 0, 8, 100);

    boolean first = true;
    int num = 1, ready = 0;
    ElapsedTime timer = new ElapsedTime(), beginning = new ElapsedTime();

    @Override
    public void start() {
        Robot.getInstance().getCamera().setPS1CamPos();
        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);
        Robot.getInstance().getFlyWheel().psPower = true;
        Robot.getInstance().fwLock.setPosition(.4);

        beginning.reset();
    }

    @Override
    public void update() {
        if (first && beginning.milliseconds() > 400) {
            if (ready < 3) {
                turn(1);
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().getCamera().setPS2CamPos();
                    Robot.getInstance().fwArm.setPosition(1);
                } else {
                    num = 2;
                    ready = 0;
                    first = false;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (num == 2 && !first) {
            if (ready < 4) {
                turn(1.2);
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().getCamera().setPS3CamPos();
                    Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT2);
                    Robot.getInstance().fwArm.setPosition(1);
                } else {
                    num = 3;
                    ready = 0;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (!first) {
            if (ready < 4) {
                turn(1.1);
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.05) {
                    ready++;
                } else {
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 200) {
                    Robot.getInstance().fwArm.setPosition(1);
                } else {
                    num = 4;
                    ready = 0;
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        }
    }

    void turn (double speedMultiplier) {
        camOrientationPIDF.p = p;
        camOrientationPIDF.i = i;
        camOrientationPIDF.d = d;
        camOrientationPIDF.kFF = f;

        double turn = Robot.getInstance().getMecanumDrive().getAngle().getTheda(AngleUnit.DEGREES) -
                Math.toDegrees(Robot.getInstance().getMecanumDrive().angleFromAuton);

        if (turn > 180) {
            turn = turn - 360;
        } else if (turn < -180) {
            turn = turn + 360;
        }
        if (turn > 180) {
            turn = turn - 360;
        } else if (turn < -180) {
            turn = turn + 360;
        }
        if (turn > 180) {
            turn = turn - 360;
        } else if (turn < -180) {
            turn = turn + 360;
        }
        turn *= -1;

        double speed = 0;
        if (Robot.getInstance().getCamera().getPosOfTarget().x == 1e10) {
            camOrientationPIDF.update(turn, 20);
            speed = CruiseLib.limitValue(camOrientationPIDF.getOutput(), -.05, -.6, .05, .6);
        } else {
            double camera = Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x);
            speed = 0.4266 * Math.pow(camera, 0.6031);
            speed = speed * (Robot.getInstance().getCamera().getPosOfTarget().x / camera);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed * speedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return num >= 4;
    }

    @Override
    public void done() {
        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.OFF);
        Robot.getInstance().getFlyWheel().psPower = false;
        Robot.getInstance().fwLock.setPosition(1);
    }
}
