package com.team9889.ftc2020.auto.actions.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShots extends Action {
    public static double p = 8, i = 0, d = 100, max = 0;
    public static double p2 = .05, i2 = 0, d2 = 2, max2 = 0;
    public static double p3 = .05, i3 = 0, d3 = 2, max3 = 0;
    public static int readyCount = 6;

    private PID yOrientationPID = new PID(1, 0, 8, 100);
    private PID camOrientationPID = new PID(1, 0, 8, 100);
    private PID angleOrientationPID = new PID(1, 0, 8, 100);

    boolean first = true;
    int num = 1, ready = 0;
    ElapsedTime timer = new ElapsedTime(), beginning = new ElapsedTime();
    Telemetry telemetry;

    double turn, y;

    public PowerShots(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        Robot.getInstance().getCamera().setPS1CamPos();
        Robot.getInstance().getCamera().setScanForGoal();
        Robot.getInstance().getMecanumDrive().setPower(0,0,0);
        Robot.getInstance().getMecanumDrive().writeAngleToFile();
        Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT1;
        Robot.getInstance().getFlyWheel().autoPower = true;
        Robot.getInstance().fwLock.setPosition(.4);
        Robot.getInstance().fwFlap.setPosition(.52);

        y = Robot.getInstance().getMecanumDrive().getAdjustedPose().getX();

        beginning.reset();
    }

    @Override
    public void update() {
//        Robot.getInstance().update();
        turn = Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES) -
                Math.toDegrees(Robot.getInstance().getMecanumDrive().angleFromAuton);

        Log.v("Turn", turn + "");

        if (first && beginning.milliseconds() > 600) {
            if (ready < readyCount) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.02 && Math.abs(turn) < .5) {
                    ready++;
                } else {
                    turn(1);
                    ready = 0;
                }
            } else if (beginning.milliseconds() > 1500) {
                if (timer.milliseconds() < 300) {
                    Robot.getInstance().getCamera().setPS2CamPos();
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 2;
                    ready = 0;
                    first = false;
                    camOrientationPID = new PID(0, 0, 0, 0);
//                    Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT2);
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (num == 2 && !first) {
            if (ready < readyCount) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.02 && Math.abs(turn) < .5) {
                    ready++;
                } else {
                    turn(1);
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 300) {
                    Robot.getInstance().getCamera().setPS3CamPos();
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 3;
                    ready = 0;
                    camOrientationPID = new PID(0, 0, 0, 0);
//                    Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT3);
                    Robot.getInstance().fwArm.setPosition(0.5);
                }
            }
        } else if (!first) {
            if (ready < readyCount) {
                timer.reset();

                if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) < 0.025 && Math.abs(turn) < .5) {
                    ready++;
                } else {
                    turn(1);
                    ready = 0;
                }
            } else {
                if (timer.milliseconds() < 300) {
                    Robot.getInstance().fwArm.setPosition(.65);
                } else {
                    num = 4;
                    ready = 0;
                    camOrientationPID = new PID(0, 0, 0, 0);
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

        angleOrientationPID.p = p2;
        angleOrientationPID.i = i2;
        angleOrientationPID.d = d2;
        angleOrientationPID.maxIntegral = max2;

        yOrientationPID.p = p3;
        yOrientationPID.i = i3;
        yOrientationPID.d = d3;
        yOrientationPID.maxIntegral = max3;

        double speed = 0;
        if (Robot.getInstance().getCamera().getPosOfTarget().x != 1e10) {
            double camera = Robot.getInstance().getCamera().getPosOfTarget().x;
            telemetry.addData("Camera", camera);
            camOrientationPID.update(camera, 0);
            speed = CruiseLib.limitValue(camOrientationPID.getOutput() / 1.2, -0.1, -1, 0.1, 1);
        }

        angleOrientationPID.update(turn, 0);
        Robot.getInstance().getMecanumDrive().turnSpeed += CruiseLib.limitValue(angleOrientationPID.getOutput(), -.05, -.6, .05, .6);

//        yOrientationPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(), y);
//        Robot.getInstance().getMecanumDrive().xSpeed += CruiseLib.limitValue(yOrientationPID.getOutput(), -.05, -.6, .05, .6);

        Robot.getInstance().getMecanumDrive().ySpeed += (speed * speedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return num >= 4;
    }

    @Override
    public void done() {
        Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;
        Robot.getInstance().getFlyWheel().autoPower = false;
        Robot.getInstance().fwLock.setPosition(1);
        Robot.getInstance().fwFlap.setPosition(.5);
    }
}
