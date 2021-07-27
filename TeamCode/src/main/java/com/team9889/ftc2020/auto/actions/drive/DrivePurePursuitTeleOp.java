//package com.team9889.ftc2020.auto.actions.drive;
//
//import android.util.Log;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
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
//public class DrivePurePursuitTeleOp extends Action {
//    ArrayList<Path> paths;
//
//    int lineNum = 0;
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
//    public DrivePurePursuitTeleOp(ArrayList<Path> paths) {
//        this.paths = paths;
//    }
//
//    public DrivePurePursuitTeleOp(ArrayList<Path> paths, PID pid) {
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
//    @Override
//    public void update() {
//        Log.i("HI", "hi");
////        Robot.getInstance().update();
//
//        Point robotPos = new Point(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
//                Robot.getInstance().getMecanumDrive().getAdjustedPose().getY());
//
//        Object[] ppObject;
//        if (lineNum + 2 >= paths.size()){ // 5
//             ppObject = pp.bestPointToFollow(paths.get(lineNum).getPoint(),
//                    paths.get(lineNum + 1).getPoint(),
//                    paths.get(lineNum + 1).getPoint(), robotPos,
//                    paths.get(lineNum + 1).getRadius(), paths.get(lineNum + 1).getRadius());
//        } else {
//            ppObject = pp.bestPointToFollow(paths.get(lineNum).getPoint(),
//                    paths.get(lineNum + 1).getPoint(),
//                    paths.get(lineNum + 2).getPoint(), robotPos,
//                    paths.get(lineNum + 1).getRadius(), paths.get(lineNum + 2).getRadius());
//        }
//
//        if (ppObject[0] == null){
//            ppObject[0] = lastPoint;
//        }
//
//        Point point = (Point) ppObject[0];
//
//        if ((boolean) ppObject[1]) {
//            lineNum++;
//        }
//
//        double xPower, yPower;
////        if (lineNum + 2 >= paths.size()) {
////            xPower = xPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
////                    paths.get(lineNum + 1).getPoint().x);
////            yPower = yPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getY(),
////                    paths.get(lineNum + 1).getPoint().y);
////        } else {
//            xPower = xPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getX(),
//                    point.x);
//            yPower = yPID.update(Robot.getInstance().getMecanumDrive().getAdjustedPose().getY(),
//                    point.y);
////        }
//
//        Log.i("Line Number : ", "" + lineNum);
//
//        double wantedAngle;
//        if (paths.get(lineNum + 1).getPose().getHeading() > 180){
//            wantedAngle = paths.get(lineNum + 1).getPose().getHeading() - 360;
//        }else if (paths.get(lineNum + 1).getPose().getHeading() < -180){
//            wantedAngle = paths.get(lineNum + 1).getPose().getHeading() + 360;
//        }else
//            wantedAngle = paths.get(lineNum + 1).getPose().getHeading();
//
//        double turn = wantedAngle - Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
//
//        if (turn > 180){
//            turn = turn - 360;
//        }else if (turn < -180){
//            turn = turn + 360;
//        }
//        turn *= -1;
//
//        double rotation = orientationPID.update(turn, 0);
//
//        double maxPower = paths.get(lineNum + 1).getMaxVelocity();
//        xPower = CruiseLib.limitValue(xPower, -.2, -maxPower, .2, maxPower);
//        yPower = CruiseLib.limitValue(yPower, -.2, -maxPower, .2, maxPower);
//        rotation = CruiseLib.limitValue(rotation, -.1, maxPower, .1, maxPower);
//
//        lastPoint = point;
//
//        Log.i("Powers : ", xPower + ", " + yPower + ", "+ rotation);
//        Robot.getInstance().getMecanumDrive().setFieldCentricAutoPower(-xPower, yPower, rotation);
//
//        Log.i("Pos", Robot.getInstance().getMecanumDrive().getCurrentPose() + "");
//        Log.i("Adj Pos", Robot.getInstance().getMecanumDrive().getAdjustedPose() + "");
//
//        Robot.getInstance().update();
//    }
//
//    @Override
//    public boolean isFinished() {
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
//        Log.i("X Counter", "" + xCounter);
//        Log.i("Y Counter", "" + yCounter);
//        Log.i("H Counter", "" + angleCounter);
//
//        return (xCounter > 3 && yCounter > 3 && angleCounter > 3 && lineNum == paths.size() - 2);
//    }
//
//    @Override
//    public void done() {
//        Robot.getInstance().getMecanumDrive().setPower(0,0 ,0);
//    }
//
//
//    Point PointSubtraction (Point point1, Point point2) {
//        return new Point(point1.x - point2.y, point1.y - point2.y);
//    }
//
//    Point PointAddition (Point point1, Point point2) {
//        return new Point(point1.x + point2.y, point1.y + point2.y);
//    }
//}
