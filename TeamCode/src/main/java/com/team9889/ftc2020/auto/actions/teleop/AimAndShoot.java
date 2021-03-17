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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.opencv.core.Point;

import java.util.ArrayList;

/**
 * Created by Eric on 8/26/2020.
 */

@Config
public class AimAndShoot extends Action {
    public static double p = 1.5, i = 0, d = 100;
//    public static double p = .5, i, d = 0;

    ArrayList<Path> paths = new ArrayList<>();

    Point posOfTarget = new Point(0, 65);

    ElapsedTime armTimer = new ElapsedTime();
    boolean extend = false;
    int ringsShot = 0;

    PurePursuit pp = new PurePursuit();
    private PID xPID = new PID(0.05, 0, 1);
    private PID yPID = new PID(0.1, 0, 1);
    private PID orientationPID = new PID(0.04, 0, 2.5);
    private PID camOrientationPID = new PID(.8, 0, 100);

    private int angleCounter = 0;
    private int xCounter = 0;
    private int yCounter = 0;
    Point lastPoint = new Point();

    boolean first = true;

    @Override
    public void start() {

    }

    boolean driveDone = false;
    @Override
    public void update() {
        if (Robot.getInstance().getFlyWheel().psPower) {
            camOrientationPID.d = 100;
            camOrientationPID.p = 11;
        } else {
            camOrientationPID.d = d;
            camOrientationPID.p = p;
        }

//        Point robotPos = new Point(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
//                Robot.getInstance().getMecanumDrive().getAdjustedPose().getY());
//
//        double angle = CalculatePose(robotPos);
//        Log.i("angle 2", angle + "");
//
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
//
//        double rotation = orientationPID.update(turn, 0);
//        rotation = CruiseLib.limitValue(rotation, .8);

//        if (Math.abs(orientationPID.getError()) < 20) {
//            Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(0, 0, rotation);
//        } else {
//        camOrientationPID.p = p;
        camOrientationPID.i = i;
//        camOrientationPID.d = d;

        double speed = 0;
        if (Robot.getInstance().getCamera().getPosOfTarget().x == 1e10) {
            camOrientationPID.update(turn, 0);
            speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.6, .05, .6);
        } else {
            camOrientationPID.update(Robot.getInstance().getCamera().getPosOfTarget().x, -.1);
            if (Robot.getInstance().getFlyWheel().psPower) {
                speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.95, -1.5, .95, 1.5);
            } else {
                speed = CruiseLib.limitValue(camOrientationPID.getOutput(), -.05, -.15, .05, .15);
            }
        }
//        .4
//            if (Math.abs(camOrientationPID.getError()) > .5) {
//                Robot.getInstance().getMecanumDrive().setFieldCentricPower(0, 0, -camOrientationPID.getOutput() * 2);
//            } else {
        if (Robot.getInstance().getFlyWheel().psPower) {
            Robot.getInstance().getMecanumDrive().turnSpeed -= (speed / 10);
        } else {
            Robot.getInstance().getMecanumDrive().turnSpeed -= (speed);
        }
//            }
//        }
    }

    double CalculatePose (Point robotPos) {
        Point offset = new Point(posOfTarget.x - robotPos.x, posOfTarget.y - robotPos.y);
        double hypotenuse = Math.sqrt(Math.pow(offset.x, 2) + Math.pow(offset.y, 2));
        double angle = Math.toDegrees(Math.asin(offset.y / hypotenuse));
        Log.i("angle", angle + "");
        return angle * (offset.x / Math.abs(offset.x));
    }

//    public boolean isDriveFinished() {
//        Pose2d tolerancePose = paths.get(paths.size() - 1).getTolerancePose();
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().x - Robot.getInstance().getMecanumDrive().getAdjustedPose().getX())
//                < Math.abs(tolerancePose.getX())) xCounter++; else xCounter = 0;
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().y - Robot.getInstance().getMecanumDrive().getAdjustedPose().getY())
//                < Math.abs(tolerancePose.getY())) yCounter++; else yCounter = 0;
//
//        if (Math.abs(orientationPID.getError()) < Math.abs(tolerancePose.getHeading()))
//            angleCounter++;
//        else angleCounter = 0;
//
//        return (xCounter > 3 && yCounter > 3 && angleCounter > 3);
//    }

    @Override
    public boolean isFinished() {
//        Pose2d tolerancePose = paths.get(paths.size() - 1).getTolerancePose();
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().x - Robot.getInstance().getMecanumDrive().getAdjustedPose().getX())
//                < Math.abs(tolerancePose.getX())) xCounter++; else xCounter = 0;
//
//        if (Math.abs(paths.get(paths.size() - 1).getPoint().y - Robot.getInstance().getMecanumDrive().getAdjustedPose().getY())
//                < Math.abs(tolerancePose.getY())) yCounter++; else yCounter = 0;

        if (Math.abs(orientationPID.getError()) < 5)
            angleCounter++;
        else angleCounter = 0;

        return (angleCounter > 3);

//        return ringsShot >= 4;
    }

    @Override
    public void done() {
        extend = false;
        ringsShot = 0;
        driveDone = false;
    }
}