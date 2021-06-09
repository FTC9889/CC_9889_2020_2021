package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.Constants;
import com.team9889.lib.control.math.cartesian.Rotation2d;
import com.team9889.lib.roadrunner.drive.DriveConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.List;

/**
 * Created by Eric on 9/7/2019.
 */

@Config
public class MecanumDrive extends Subsystem {
    public double x, y, xSpeed, ySpeed, turnSpeed;

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(12, 0, .5);
    public PIDFController headingController = new PIDFController(HEADING_PID);

    public Pose2d currentPose = new Pose2d();
    public Rotation2d gyroAngle = new Rotation2d();
    private double Right_Position_Offset = 0, Left_Position_Offset = 0, Y_Position_Offset = 0;
    public double angleFromAuton = 0;

    ElapsedTime timer = new ElapsedTime();

    public Odometry odometry;
    Pose2d velocityPose = new Pose2d(0, 0, 0),
            lastPoseOfRobotBeforeDriftCalc = new Pose2d(0, 0, 0),
            driftCalc = new Pose2d(0, 0, 0);

    private String filename = "gyro.txt";

    public boolean first = true, updated = false;

    @Override
    public void init(boolean auto) {
        if(auto) {

        } else {
//            readAngleFromFile();
        }

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        odometry = new Odometry();
//        odometry.reverseLeftEncoder();
//        odometry.reverseNormalEncoder();

        timer.reset();
        lastPoseOfRobotBeforeDriftCalc = currentPose;
        driftCalc = new Pose2d(0, 0, 0);
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Left Encoder", "" + Robot.getInstance().passThrough.getPosition());
        telemetry.addData("Right Encoder", "" + Robot.getInstance().frontIntake.getPosition());
        telemetry.addData("Side Encoder", "" + Robot.getInstance().backIntake.getPosition());

        telemetry.addData("Pos : ", getCurrentPose() + "");
        Log.i("Pos", getCurrentPose() + "");
        Log.i("Adj Pos", getAdjustedPose() + "");

//        telemetry.addData("Side Encoder", Robot.getInstance().intakeRight.getPosition());

//        telemetry.addData("Pose of Robot", getCurrentPose().toString());
//        telemetry.addData("Adjusted Pose of Robot", getAdjustedPose().toString());
//        telemetry.addData("Velocity Pose of Robot", velocityPose);
//
//        Log.i("Right Offset", "" + Right_Position_Offset);
//        Log.i("Left Offset", "" + Left_Position_Offset);
//        Log.i("Side Offset", "" + Y_Position_Offset);
    }

    @Override
    public void update() {
        odometry.update();

//        double adjustValue = 0;
//        double adjustValue = 0.00009;
//        if(timer.milliseconds() > 0)
//            velocityPose = currentPose.minus(lastPoseOfRobotBeforeDriftCalc).div(timer.seconds()).times(adjustValue);
//        else
//            velocityPose = currentPose.minus(lastPoseOfRobotBeforeDriftCalc).div(20 / 1000).times(adjustValue);
//        lastPoseOfRobotBeforeDriftCalc = currentPose;
//        timer.reset();
//        driftCalc = driftCalc.plus(velocityPose);
//
//        if (updated) {
////            setCurrentPose(new Pose2d(odometry.getPoseEstimate().getX(),
////                    odometry.getPoseEstimate().getY(),
////                    -gyroAngle.getTheda(AngleUnit.RADIANS)));
//            setCurrentPose(new Pose2d(odometry.getPoseEstimate().getX(),
//                    odometry.getPoseEstimate().getY(),
//                    odometry.getPoseEstimate().getHeading()));
//        }
//        odometry.update();

        updated = true;

//        Robot.getInstance().rr.setWeightedDrivePower(new Pose2d(xSpeed, ySpeed, turnSpeed));

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

    public Pose2d getCurrentPose() {
//        return currentPose.plus(driftCalc);
//        return currentPose;
        return odometry.getPoseEstimate();
    }
    public Pose2d getAdjustedPose(){
        return currentPose.plus(driftCalc);
    }

    public void setCurrentPose(Pose2d pose) {
        resetOdometryEncoders();
        odometry.setPoseEstimate(pose);
        currentPose = odometry.getPoseEstimate();
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
//        FileWriter angleWriter = new FileWriter(filename);
//        angleWriter.write(gyroAngle.getTheda(AngleUnit.RADIANS));
//        angleWriter.close();

        angleFromAuton = gyroAngle.getTheda(AngleUnit.RADIANS);
    }

    public void setFieldCentricPower(double x, double y, double rotation, boolean blue){
        double angle = getAngle().getTheda(AngleUnit.RADIANS) + Math.toRadians(90);

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

        turnSpeed += headingInput;

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
//        Robot.getInstance().rr.getLocalizer().update();
    }

}

class Odometry extends ThreeTrackingWheelLocalizer {

    private static final double LATERAL_DISTANCE = 4.4457;
    private static final double FORWARD_OFFSET = 0.241;

    Odometry() {
        super(Arrays.asList(
                new Pose2d(FORWARD_OFFSET, LATERAL_DISTANCE, Math.toRadians(0)),
                new Pose2d(FORWARD_OFFSET, -LATERAL_DISTANCE, Math.toRadians(0)),
//                new Pose2d(-0.1875, 0.09375, Math.toRadians(90))
//                new Pose2d(-0.375, -1.125, Math.toRadians(90))
                new Pose2d(0.316, 0.820, Math.toRadians(90))
        ));
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                Robot.getInstance().getMecanumDrive().Left_OdometryPosition(),
                Robot.getInstance().getMecanumDrive().Right_OdometryPosition(),
                Robot.getInstance().getMecanumDrive().Y_OdometryPosition()
        );
    }
}
