package com.team9889.ftc2020.auto.actions.flywheel;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.PIDF;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShotsAuto extends Action {

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
            if (ready < 10) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.01) {
                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
                    ready++;
                } else {
                    turn(1.2);
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
            if (ready < 10) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.01) {
                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
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
            if (ready < 10) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.01) {
                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
                    ready++;
                } else {
                    turn(1.2);
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
            speed = -CruiseLib.limitValue(camOrientationPID.getOutput() / 1.2, -.1, -.6, .1, .6);
        } if (Math.abs(turn) > 25) {
            camOrientationPID.update(turn, 20);
            speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.6, .05, .6);
        }

        Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, (speed * speedMultiplier));
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


//    public static double p = .08, i = 0.00000000001, d = .5, max = .1;
//
//    private PIDF camOrientationPID = new PIDF(1, 0, 8, 100);
//
//    boolean first = true;
//    int num = 1, ready = 0;
//    ElapsedTime timer = new ElapsedTime(), beginning = new ElapsedTime();
//
//    double turn;
//
//    @Override
//    public void start() {
//        Robot.getInstance().getCamera().setPS1CamPosAuto();
//        Robot.getInstance().getCamera().setScanForGoal();
//        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOTAUTO1);
//        Robot.getInstance().getFlyWheel().psPower = true;
//        Robot.getInstance().fwLock.setPosition(.4);
//
//        beginning.reset();
//
//        Log.v("Power Shots", "");
//    }
//
//    @Override
//    public void update() {
//        Robot.getInstance().update();
//
//        turn = -(Robot.getInstance().getMecanumDrive().getAngle().getTheda(AngleUnit.DEGREES) -
//                Math.toDegrees(Robot.getInstance().getMecanumDrive().angleFromAuton));
//
//        if (first && beginning.milliseconds() > 600) {
//            if (ready < 10) {
//                if (Math.abs(turn + 12) < .5) {
//                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
//                    ready++;
//                } else {
//                    turn(1, -12);
//                    timer.reset();
//                    ready = 0;
//                }
//                timer.reset();
//            } else {
//                if (timer.milliseconds() < 200) {
//                    Robot.getInstance().getCamera().setPS2CamPosAuto();
//                    Robot.getInstance().fwArm.setPosition(1);
//                } else {
//                    num = 2;
//                    ready = 0;
//                    first = false;
//                    Robot.getInstance().fwArm.setPosition(0.5);
//                    Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOTAUTO2);
//                }
//            }
//        } else if (num == 2 && !first) {
//            if (ready < 10) {
//                if (Math.abs(turn + 8) < .5) {
//                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
//                    ready++;
//                } else {
//                    turn(1, -8);
//                    timer.reset();
//                    ready = 0;
//                }
//                timer.reset();
//            } else {
//                if (timer.milliseconds() < 200) {
//                    Robot.getInstance().getCamera().setPS3CamPosAuto();
//                    Robot.getInstance().fwArm.setPosition(1);
//                } else {
//                    num = 3;
//                    ready = 0;
//                    Robot.getInstance().fwArm.setPosition(0.5);
//                }
//            }
//        } else if (!first) {
//            if (ready < 10) {
//                if (Math.abs(turn + 2) < .5) {
//                    Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
//                    ready++;
//                } else {
//                    turn(1, -2);
//                    timer.reset();
//                    ready = 0;
//                }
//                timer.reset();
//            } else {
//                if (timer.milliseconds() < 200) {
//                    Robot.getInstance().fwArm.setPosition(1);
//                } else {
//                    num = 4;
//                    ready = 0;
//                    Robot.getInstance().fwArm.setPosition(0.5);
//                }
//            }
//        }
//    }
//
//    void turn (double speedMultiplier, double angle) {
//        camOrientationPID.p = p;
//        camOrientationPID.i = i;
//        camOrientationPID.d = d;
//        camOrientationPID.maxIntegral = max;
//
//        double speed = 0;
////        if (Math.abs(turn) > 25) {/
////            camOrientationPID.update(turn, 20);
////            speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.6, .05, .6);
////        } else if (Robot.getInstance().getCamera().getPosOfTarget().x != 1e10) {
////            double camera = Robot.getInstance().getCamera().getPosOfTarget().x;
////            camOrientationPID.update(camera, 0);
//            camOrientationPID.update(turn, angle);
//            speed = -CruiseLib.limitValue(camOrientationPID.getOutput(), -.12, -.6, .12, .6);
////        }
//
//        Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, speed * speedMultiplier);
////        Robot.getInstance().getMecanumDrive().turnSpeed += speed * speedMultiplier * 1.1;
//    }
//
//    @Override
//    public boolean isFinished() {
//        return num >= 4;
//    }
//
//    @Override
//    public void done() {
//        Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, 0);
//        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.OFF);
//        Robot.getInstance().getFlyWheel().psPower = false;
//        Robot.getInstance().fwLock.setPosition(1);
//        Robot.getInstance().fwArm.setPosition(0.5);
//    }
}
