package com.team9889.ftc2020.auto.actions.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.PurePursuit;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;

import java.util.ArrayList;

/**
 * Created by Eric on 8/26/2020.
 */

@Config
public class AimAndShoot extends Action {
    public static double p = .7, i = 0, d = 40, f = 0;
    public static double multiplier, multiplier12 = .95, multiplier11 = 1, multiplier10 = 1.05;
//    public static double p = .5, i, d = 0;

    ArrayList<Path> paths = new ArrayList<>();

    Point posOfTarget = new Point(0, 65);

    boolean extend = false;
    int ringsShot = 0;

    private PID camOrientationPID;

    private int angleCounter = 0;
    private int xCounter = 0;
    private int yCounter = 0;
    Point lastPoint = new Point();

    boolean first = true;

    @Override
    public void start() {
        camOrientationPID = new PID(0.4, 0.0000001, 0.7, 1);
    }

    boolean driveDone = false;
    @Override
    public void update() {
        if (Robot.getInstance().result > 12) {
            multiplier = multiplier12;
        } else if (Robot.getInstance().result <= 12 && Robot.getInstance().result > 11) {
            multiplier = multiplier11;
        } else if (Robot.getInstance().result <= 11) {
            multiplier = multiplier10;
        }

        camOrientationPID.p = p;
        camOrientationPID.i = i;
        camOrientationPID.d = d;
        camOrientationPID.maxIntegral = f;

        double turn = Robot.getInstance().getMecanumDrive().getAdjustedPose().getHeading() -
                Math.toDegrees(Robot.getInstance().getMecanumDrive().angleFromAuton);

        double camera = Robot.getInstance().getCamera().getPosOfTarget().x;
        double speed = 0;
        if (Robot.getInstance().getCamera().getPosOfTarget().x != 1e10) {
            camOrientationPID.update(camera, 0);
            speed = -CruiseLib.limitValue(camOrientationPID.getOutput() / 1.2, -.12, -.6, .12, .6);
//            if (Math.abs(camera) < .2) {
//                speed = (camera / Math.abs(camera)) * .15;
//            } else {
//                speed = (camera / Math.abs(camera)) * .25;
//            }
        } else if (Math.abs(turn) > 5) {
            camOrientationPID.update(turn / 20, 0);
            speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.6, .05, .6);
        }

        Robot.getInstance().getMecanumDrive().turnSpeed += (speed);
//        Robot.getInstance().getMecanumDrive().turnSpeed += (camera / Math.abs(camera)) * .2;
    }

    @Override
    public boolean isFinished() {
//        Pose2d tolerancePose = paths.get(paths.size() - 1).getTolerancePose();
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().x - Robot.getInstance().getMecanumDrive().getAdjustedPose().getX())
//                < Math.abs(tolerancePose.getX())) xCounter++; else xCounter = 0;
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().y - Robot.getInstance().getMecanumDrive().getAdjustedPose().getY())
//                < Math.abs(tolerancePose.getY())) yCounter++; else yCounter = 0;
//
//        if (Math.abs(camOrientationPID.getError()) < 5)
//            angleCounter++;
//        else angleCounter = 0;

//        return (angleCounter > 3);

        return false;

//        return ringsShot >= 4;
    }

    @Override
    public void done() {
        extend = false;
        ringsShot = 0;
        driveDone = false;
    }
}