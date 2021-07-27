//package com.team9889.ftc2020.auto.actions.teleop;
//
//import android.util.Log;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.team9889.ftc2020.auto.actions.Action;
//import com.team9889.ftc2020.subsystems.Robot;
//import com.team9889.lib.CruiseLib;
//import com.team9889.lib.control.Path;
//import com.team9889.lib.control.PurePursuit;
//import com.team9889.lib.control.controllers.PID;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.opencv.core.Point;
//
//import java.util.ArrayList;
//
///**
// * Created by Eric on 8/26/2020.
// */
//public class DriveAndShoot extends Action {
//    ArrayList<Path> paths;
//
//    ElapsedTime armTimer = new ElapsedTime();
//    boolean extend = false;
//    int ringsShot = 0;
//
//    PurePursuit pp = new PurePursuit();
//    private PID xPID = new PID(0.05, 0, 1);
//    private PID yPID = new PID(0.1, 0, 1);
//    private PID orientationPID = new PID(0.04, 0, 2.5);
//
//    private int angleCounter = 0;
//    private int xCounter = 0;
//    private int yCounter = 0;
//    Point lastPoint = new Point();
//
//    public DriveAndShoot(ArrayList<Path> paths) {
//        this.paths = paths;
//    }
//
//    public DriveAndShoot(ArrayList<Path> paths, PID pid) {
//        this.paths = paths;
//        this.orientationPID = pid;
//    }
//
//    @Override
//    public void start() {
//        paths.add(0, new Path(Robot.getInstance().getMecanumDrive().getCurrentPose(),
//                paths.get(0).getTolerancePose(), paths.get(0).getRadius(), paths.get(0).getMaxVelocity()));
//    }
//
//    boolean driveDone = false;
//    @Override
//    public void update() {
//        if (!driveDone) {
//            Point robotPos = new Point(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
//                    Robot.getInstance().getMecanumDrive().getAdjustedPose().getY());
//
//            Object[] ppObject;
//            ppObject = pp.bestPointToFollow(paths.get(0).getPoint(),
//                    paths.get(1).getPoint(),
//                    paths.get(1).getPoint(), robotPos,
//                    paths.get(1).getRadius(), paths.get(1).getRadius());
//
//            if (ppObject[0] == null) {
//                ppObject[0] = lastPoint;
//            }
//
//            Point point = (Point) ppObject[0];
//
//            double xPower = xPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
//                    point.x);
//            double yPower = yPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getY(),
//                    point.y);
//
//            double wantedAngle;
//            if (paths.get(1).getPose().getHeading() > 180) {
//                wantedAngle = paths.get(1).getPose().getHeading() - 360;
//            } else if (paths.get(1).getPose().getHeading() < -180) {
//                wantedAngle = paths.get(1).getPose().getHeading() + 360;
//            } else
//                wantedAngle = paths.get(1).getPose().getHeading();
//
//            double turn = wantedAngle - Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
//
//            if (turn > 180) {
//                turn = turn - 360;
//            } else if (turn < -180) {
//                turn = turn + 360;
//            }
//            turn *= -1;
//
//            double rotation = orientationPID.update(turn, 0);
//
//            double maxPower = paths.get(1).getMaxVelocity();
//            xPower = CruiseLib.limitValue(xPower, maxPower, -maxPower);
//            yPower = CruiseLib.limitValue(yPower, maxPower, -maxPower);
//            rotation = CruiseLib.limitValue(rotation, paths.get(1).getMaxTurnVelocity());
//
//            lastPoint = point;
//
//            Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(-xPower, yPower, rotation);
//
//            driveDone = isDriveFinished();
//        } else {
//            Robot.getInstance().getMecanumDrive().setPower(0,0 ,0);
//
//            Log.i("else", "");
//                if (armTimer.milliseconds() > 300) {
//                    Log.i("timer", "");
//                    if (extend) {
//                        Robot.getInstance().fwArm.setPosition(0.7);
//                        extend = false;
//                        ringsShot++;
//                    } else {
//                        Robot.getInstance().fwArm.setPosition(0);
//                        extend = true;
//                    }
//                    Log.i("extend", "");
//
//                    armTimer.reset();
//                }
//
//            Log.i("rings", ringsShot + "");
//        }
//        }
//
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
//
//    @Override
//    public boolean isFinished() {
//        return ringsShot >= 4;
//    }
//
//    @Override
//    public void done() {
//        extend = false;
//        ringsShot = 0;
//        driveDone = false;
//    }
//}