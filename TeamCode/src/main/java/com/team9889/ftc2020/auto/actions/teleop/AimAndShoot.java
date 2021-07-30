package com.team9889.ftc2020.auto.actions.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 8/26/2020.
 */

@Config
public class AimAndShoot extends Action {
    public static double p = .5, d = 30;
    public static double p2 = 1.2, d2 = 50;

    private PID camOrientationPID = new PID(0.5, 0, 30);
    private PID gyroOrientationPID = new PID(12, 0, .5);

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(12, 0, .5);
    public PIDFController headingController = new PIDFController(HEADING_PID);

    @Override
    public void start() {}

    @Override
    public void update() {
        updatePIDValues();

        double cameraTarget = Robot.getInstance().getCamera().getPosOfTarget().x;
        double turn = Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.RADIANS);

        double speed = 0;
        if (Math.abs(turn) > Math.toRadians(25)) {
            speed = gyroOrientationPID.update(turn, 0);
            Log.i("Turn", "" + turn);
        }
        else if (Math.abs(Robot.getInstance().getCamera().getPosOfTarget().x) >= 0.05){
            speed = -camOrientationPID.update(cameraTarget, 0);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed);
    }

    void updatePIDValues () {
        camOrientationPID.p = p;
        camOrientationPID.d = d;

        gyroOrientationPID.p = p2;
        gyroOrientationPID.d = d2;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {
    }
}