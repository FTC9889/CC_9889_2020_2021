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

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(14, 0, .8);
    public PIDFController headingController = new PIDFController(HEADING_PID);

    public Rotation2d gyroAngle = new Rotation2d();
    private double Right_Position_Offset = 0, Left_Position_Offset = 0, Y_Position_Offset = 0;
    public double angleFromAuton = 0;

    ElapsedTime timer = new ElapsedTime();

    private String filename = "gyro.txt";

    @Override
    public void init(boolean auto) {
        if(auto) {
            Robot.getInstance().leftArm.setPosition(0);
            Robot.getInstance().rightArm.setPosition(1);
        } else {
            angleFromAuton = Math.toDegrees(Constants.pose.getHeading());
        }

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        timer.reset();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        Robot.getInstance().rr.getLocalizer().getPoseEstimate();
    }

    @Override
    public void update() {
        getAngle();

        xSpeed = 0;
        ySpeed = 0;
        turnSpeed = 0;
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
        Pose2d poseEstimate = Robot.getInstance().rr.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(0, 0);
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();

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

        headingInput = CruiseLib.limitValue(headingInput, 0.4);
        Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(0, 0, headingInput));

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        Robot.getInstance().rr.getLocalizer().update();
    }

    public void psTurn (Vector2d targetPosition) {
        Robot.getInstance().rr.getLocalizer().setPoseEstimate(new Pose2d(-8, -16, gyroAngle.getTheda(AngleUnit.RADIANS)));
        Pose2d poseEstimate = Robot.getInstance().rr.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(0, 0);
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double headingInput = (headingController.update(poseEstimate.getHeading())
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        turnSpeed += headingInput / 26;

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
//        Robot.getInstance().rr.getLocalizer().update();
    }
}