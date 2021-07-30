package com.team9889.ftc2020.auto.actions.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 4/5/2021.
 */

@Config
public class PowerShots extends Action {
    public static double p = 8, i = 0, d = 100, max = 0;
    public static int readyCount = 30;

    boolean firstShot = true;
    int ready = 0;
    ElapsedTime beginning = new ElapsedTime(), timer = new ElapsedTime();

    boolean first, second, third, auto;

    public PowerShots(){
        first = true;
        second = true;
        third = true;
        auto = false;
    }

    public PowerShots(boolean auto){
        first = true;
        second = true;
        third = true;
        this.auto = auto;
    }

    public PowerShots(boolean first, boolean second, boolean third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    @Override
    public void start() {
        firstShot = true;

        Robot.getInstance().getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT2;

        Robot.getInstance().getCamera().setScanForGoal();
        Robot.getInstance().getMecanumDrive().setPower(0,0,0);
        Robot.getInstance().getFlyWheel().autoPower = true;
        Robot.getInstance().getFlyWheel().unlocked();
        Robot.getInstance().getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;

        beginning.reset();
    }

    @Override
    public void update() {
//        if (first && beginning.milliseconds() > 600) {
        if (first) {
            if (ready < readyCount) {
                double angle = -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
                if (angle < 0) {
                    angle += 360;
                }

                Log.i("Angle", "" + (Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle));

                if (Math.abs(Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle) < 1) {
                    Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    ready++;
                } else if (ready < readyCount / 2){
                    ready = 0;
                    Robot.getInstance().getMecanumDrive().turn(new Vector2d(73, -22), new Vector2d(0, 0));
                }
            } else if (beginning.milliseconds() > 1500) {
                boolean ringShot = Robot.getInstance().getFlyWheel().shootRing();
                if (ringShot) {
                    first = false;
                    ready = 0;
                    firstShot = false;
                    Robot.getInstance().getMecanumDrive().theta = 1000;
                }
            }
        } else if (third) {
            if (ready < readyCount && second) {
                double angle = -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
                if (angle < 0) {
                    angle += 360;
                }

                Log.i("Angle", "" + (Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle));

                if (Math.abs(Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle) < 1) {
                    Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    ready++;
                } else if (ready < readyCount / 2) {
                    ready = 0;
                    Robot.getInstance().getMecanumDrive().turn(new Vector2d(73, 0), new Vector2d(0, 0));
                }
            } else {
                boolean ringShot = Robot.getInstance().getFlyWheel().shootRing();
                if (ringShot) {
                    third = false;
                    ready = 0;
                    Robot.getInstance().getMecanumDrive().theta = 1000;
                }
            }
        } else if (second) {
            if (ready < readyCount) {
                double angle = -Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
                if (angle < 0) {
                    angle += 360;
                }

                Log.i("Angle", "" + (Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle));

                if (Math.abs(Math.toDegrees(Robot.getInstance().getMecanumDrive().theta) - angle) < 1) {
                    Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    ready++;
                } else if (ready < readyCount / 2) {
                    ready = 0;
                    Robot.getInstance().getMecanumDrive().turn(new Vector2d(73, -9), new Vector2d(0, 0));
                }
            } else {
                boolean ringShot = Robot.getInstance().getFlyWheel().shootRing();
                if (ringShot) {
                    second = false;
                    ready = 0;
                    Robot.getInstance().getMecanumDrive().theta = 1000;
                }
            }
        }

        if (auto) {
            while (timer.milliseconds() < 20) {}
            timer.reset();
        }

//        Robot.getInstance().update();
    }

    @Override
    public boolean isFinished() {
        return !first && !second && !third;
    }

    @Override
    public void done() {
        Robot.getInstance().getFlyWheel().autoPower = false;
        Robot.getInstance().getFlyWheel().locked();
        Robot.getInstance().getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;
    }
}
