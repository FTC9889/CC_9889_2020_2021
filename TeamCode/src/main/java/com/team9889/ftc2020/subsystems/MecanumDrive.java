package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.Constants;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.math.cartesian.Rotation2d;
import com.team9889.lib.roadrunner.drive.DriveConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

/**
 * Created by Eric on 9/7/2019.
 */

@Config
public class MecanumDrive extends Subsystem {
    public double x, y, xSpeed, ySpeed, turnSpeed;

    double[] turnError = new double[5];

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7.5, 0.0015, 0.55);
    public PIDFController headingController = new PIDFController(HEADING_PID);

    public double theta;
    public boolean resetPos = false, resetFirst = true;

    public Rotation2d gyroAngle = new Rotation2d();
    private double Right_Position_Offset = 0, Left_Position_Offset = 0, Y_Position_Offset = 0;
    public double angleFromAuton = 0;

    ElapsedTime timer = new ElapsedTime();

    private String filename = "gyro.txt";

    public boolean dontMove = false;

    @Override
    public void init(boolean auto) {
        if(auto) {
            Robot.getInstance().leftArm.setPosition(0);
            Robot.getInstance().rightArm.setPosition(1);
        } else if (Constants.pose != null) {
            Robot.getInstance().rr.setPoseEstimate(Constants.pose);
            Robot.getInstance().rr.update();
            angleFromAuton = Constants.pose.getHeading();
        }

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        timer.reset();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Pos", Robot.getInstance().rr.getLocalizer().getPoseEstimate());

        telemetry.addData("Gyro", gyroAngle.getTheda(AngleUnit.DEGREES) - angleFromAuton);

        telemetry.addData("Error", theta - angle);
    }

    @Override
    public void update() {
        getAngle();

//        if (!dontMove) {
            setFieldCentricPower(xSpeed, ySpeed, turnSpeed, Robot.getInstance().blue);
            xSpeed = 0;
            ySpeed = 0;
            turnSpeed = 0;
//        }
    }

    @Override
    public void stop() {
        Robot.getInstance().fLDrive.setPower(0);
        Robot.getInstance().fRDrive.setPower(0);
        Robot.getInstance().bLDrive.setPower(0);
        Robot.getInstance().bRDrive.setPower(0);
    }

    public void resetOdometryEncoders() {
        Right_Position_Offset = Right_OdometryPosition();
        Left_Position_Offset = Left_OdometryPosition();
        Y_Position_Offset = Y_OdometryPosition();
    }

    public double Right_OdometryPosition() {
        return (-Robot.getInstance().frontIntake.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Right_Position_Offset;
    }

    public double Left_OdometryPosition() {
        return (-Robot.getInstance().passThrough.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Left_Position_Offset;
    }

    public double Y_OdometryPosition() {
        return (Robot.getInstance().backIntake.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Y_Position_Offset;
    }

    public Rotation2d getAngle(){
        try {
            gyroAngle.setTheda(-Robot.getInstance().imu.getNormalHeading(), AngleUnit.DEGREES);
            return gyroAngle;
        } catch (Exception e){
            return new Rotation2d(0, AngleUnit.DEGREES);
        }
    }

    public void writeAngleToFile() {
        angleFromAuton = gyroAngle.getTheda(AngleUnit.RADIANS);
    }

    public void setFieldCentricPower(double x, double y, double rotation, boolean blue){
        double angle = gyroAngle.getTheda(AngleUnit.RADIANS) + Math.toRadians(90);

        if (blue) {
            x = -x;
            y = -y;
        }

        double angleFromAuto = Robot.getInstance().getMecanumDrive().angleFromAuton;
        double xMod = x * Math.cos(angle - angleFromAuto) - y * Math.sin(angle - angleFromAuto);
        double yMod = x * Math.sin(angle - angleFromAuto) + y * Math.cos(angle - angleFromAuto);

        Log.v("Drive", "X:" + xMod + "Y:" + yMod + "Rotation:" + rotation);
        setPower(xMod, yMod, rotation);
    }

    public void setFieldCentricAutoPower(double x, double y, double rotation){
        double xMod = x * gyroAngle.cos() - y * gyroAngle.sin();
        double yMod = x * gyroAngle.sin() + y * gyroAngle.cos();
        setPower(xMod, yMod, rotation);
    }

    public void setPower(double leftStickX, double leftStickY, double rightStickX){
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double rightX = rightStickX;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        Robot.getInstance().fLDrive.setPower(v1);
        Robot.getInstance().fRDrive.setPower(v2);
        Robot.getInstance().bLDrive.setPower(v3);
        Robot.getInstance().bRDrive.setPower(v4);
    }


    public double robotAngleToTarget(Vector2d target, Pose2d curPos){
        double tgtAngle = Math.PI/2.0 - Math.atan2(target.getX()-curPos.getX(),target.getY()-curPos.getY());
        return tgtAngle;
    }

    public double robotDistanceToTarget(Point target, Pose2d curPos) {
        double dist = Math.sqrt(Math.pow(target.x - curPos.getX(), 2) + Math.pow(target.y - curPos.getY(), 2));
        return dist;
    }

    public void turn (Vector2d targetPosition) {
        Pose2d poseEstimate = Robot.getInstance().rr.getLocalizer().getPoseEstimate();

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(0, 0);
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        theta = difference.angle();

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double headingInput = (headingController.update(poseEstimate.getHeading())
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

//        turnSpeed += headingInput;

//        headingInput = CruiseLib.limitValue(headingInput, 0.4);
        headingInput = CruiseLib.limitValue(headingInput, -0.1, -1, 0.1, 1);
        Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, headingInput));

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        Robot.getInstance().rr.getLocalizer().update();
    }

    public double angle;
    public void turn (Vector2d targetPosition, Vector2d input) {
        Pose2d poseEstimate = Robot.getInstance().rr.getLocalizer().getPoseEstimate();
        poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), -gyroAngle.getTheda(AngleUnit.RADIANS));
        Robot.getInstance().rr.update();

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = input;
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        theta = difference.angle();

//        if (Math.toDegrees(theta) < 180) {
//            Log.i("Added", "" + 0.25 * Math.toDegrees(theta));
//            theta += Math.toRadians(0.25 * Math.toDegrees(theta));
//        } else {
//            Log.i("Added", "" + (45 - (0.25 * (Math.toDegrees(theta) - 180))));
//            theta -= Math.toRadians((45 - (0.25 * (Math.toDegrees(theta) - 180))));
//        }

//        Log.i("Theta", "" + Math.toDegrees(theta));

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        angle = -gyroAngle.getTheda(AngleUnit.DEGREES);
        if (angle < 0) {
            angle += 360;
        }

        double headingInput = (headingController.update(Math.toRadians(angle))
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        Log.i("Error", "" + headingController.getLastError());

        int numOfBad = 0;
        for (int i = 0; i < turnError.length; i++) {
//            Log.i("Turn Error " + i, "" + (turnError[i] - headingController.getLastError()));
            if (Math.abs(turnError[i] - headingController.getLastError()) < Math.toRadians(.1))
                numOfBad += 1;

            if (i + 1 < turnError.length) {
                turnError[i] = turnError[i + 1];
            } else {
                turnError[i] = headingController.getLastError();
            }
        }

//        Log.i("BAD", "" + numOfBad);

        if (numOfBad == 5) {
            Log.i("BAD", "");
            headingInput += .5;
        }

//        turnSpeed += headingInput;

        headingInput = CruiseLib.limitValue(headingInput, -0.1, -1, 0.1, 1);
        Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, headingInput));
//        turnSpeed = headingInput;

        // Update the heading controller with our current heading
        headingController.update(Math.toRadians(angle));

        // Update he localizer
//        Robot.getInstance().rr.getLocalizer().update();
    }



    public void turn (Vector2d targetPosition, boolean rr) {
        Pose2d poseEstimate = Robot.getInstance().rr.getLocalizer().getPoseEstimate();
        poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), -gyroAngle.getTheda(AngleUnit.RADIANS));
        Robot.getInstance().rr.update();

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(0, 0);
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        theta = difference.angle();
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double angle = -gyroAngle.getTheda(AngleUnit.DEGREES);
        if (angle < 0) {
            angle += 360;
        }

        double headingInput = (headingController.update(Math.toRadians(angle))
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        Log.i("Error", "" + headingController.getLastError());

        int numOfBad = 0;
        for (int i = 0; i < turnError.length; i++) {
//            Log.i("Turn Error " + i, "" + (turnError[i] - headingController.getLastError()));
            if (Math.abs(turnError[i] - headingController.getLastError()) < Math.toRadians(.1))
                numOfBad += 1;

            if (i + 1 < turnError.length) {
                turnError[i] = turnError[i + 1];
            } else {
                turnError[i] = headingController.getLastError();
            }
        }

        if (numOfBad == 5) {
            Log.i("BAD", "");
            headingInput += .5;
        }

//        turnSpeed += headingInput;

        headingInput = CruiseLib.limitValue(headingInput, -0.1, -.4, 0.1, .4);
//        Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, headingInput));

        Log.i("Turn", "" + (theta - angle));
        Robot.getInstance().rr.turnAsync(theta - angle);

//        turnSpeed = headingInput;

        // Update the heading controller with our current heading
        headingController.update(Math.toRadians(angle));

        // Update he localizer
//        Robot.getInstance().rr.getLocalizer().update();
    }


    Pose2d moveStartPos = new Pose2d(0, 0, 0);

    public void move () {
        dontMove = true;

//        double side = Robot.getInstance().blue ? -1 : 1;
//        Robot.getInstance().rr.followTrajectoryAsync(
//                Robot.getInstance().rr.trajectoryBuilder(Robot.getInstance().rr.getPoseEstimate())
//                .strafeTo(moveStartPos.vec().plus(new Vector2d(0, 8 * side)))
//                        .build());
//
//        Robot.getInstance().rr.update();

        if (Math.abs(Robot.getInstance().rr.getPoseEstimate().getY() - moveStartPos.getY()) < 6) {
            xSpeed = 0;
            ySpeed += .7;
        }
    }

    public void setInitPowerShotPos () {
        moveStartPos = Robot.getInstance().rr.getPoseEstimate();
        dontMove = false;
    }
}