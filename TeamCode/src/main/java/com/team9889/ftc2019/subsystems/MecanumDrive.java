package com.team9889.ftc2019.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.team9889.ftc2019.Constants;
import com.team9889.lib.android.FileReader;
import com.team9889.lib.android.FileWriter;
import com.team9889.lib.control.math.cartesian.Rotation2d;
import com.team9889.lib.control.kinematics.Odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 9/7/2019.
 */

public class MecanumDrive extends Subsystem {
    public double x, y, xSpeed, ySpeed;

    public Pose2d currentPose = new Pose2d();
    public Rotation2d gyroAngle = new Rotation2d();
    private double Right_Position_Offset = 0, Left_Position_Offset = 0, Y_Position_Offset = 0;
    public double angleFromAuton = 0;

    public Odometry odometry;

    private String filename = "gyro.txt";

    public boolean first = true, updated = false;

    @Override
    public void init(boolean auto) {
        if(auto) {

        } else {
//            readAngleFromFile();
        }

//        odometry.reverseLeftEncoder();
//        odometry.reverseNormalEncoder();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
//        Log.i("Pose Of Robot", "" + getCurrentPose());
//        telemetry.addData("Left Encoder", "" + Robot.getInstance().leftLift.getPosition());
//        telemetry.addData("Right Encoder", "" + Robot.getInstance().intakeLeft.getPosition());
//        telemetry.addData("Side Encoder", "" + Robot.getInstance().intakeRight.getPosition());

//        telemetry.addData("x", odometry.returnXCoordinate());
//        telemetry.addData("Y", odometry.returnYCoordinate());
//        telemetry.addData("Angle", odometry.returnOrientation());

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
//        odometry.update();

//        if (updated) {
//            setCurrentPose(new Pose2d(odometry.getPoseEstimate().getX(),
//                    odometry.getPoseEstimate().getY(),
//                    gyroAngle.getTheda(AngleUnit.RADIANS)));
//        }

//        odometry.update();

        updated = true;
    }

    private void resetOdometryEncoders() {
        Right_Position_Offset = Right_OdometryPosition();
        Left_Position_Offset = Left_OdometryPosition();
        Y_Position_Offset = Y_OdometryPosition();
    }

    public double Right_OdometryPosition() {
        return (Robot.getInstance().intakeLeft.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Right_Position_Offset;
    }

    public double Left_OdometryPosition() {
        return (Robot.getInstance().fRDrive.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Left_Position_Offset;
    }

    public double Y_OdometryPosition() {
        return (Robot.getInstance().intakeRight.getPosition() * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO) - Y_Position_Offset;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
//    public Pose2d getAdjustedPose(){
//        return currentPose.plus(driftCalc);
//    }

//    public void setCurrentPose(Pose2d pose) {
//        resetOdometryEncoders();
//        odometry.setPoseEstimate(pose);
//        currentPose = odometry.getPoseEstimate();
//    }

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

//    public void readAngleFromFile() {
//        try {
//            FileReader angleReader = new FileReader(filename);
//            String[] rows = angleReader.lines();
//            if (rows[rows.length - 1] != null) {
//                String value = rows[rows.length - 1];
//                angleFromAuton = Double.parseDouble(value);
//                angleReader.close();
//            } else{
//                angleFromAuton = 0;
//            }
//        } catch (NumberFormatException e) {
//            angleFromAuton = 0;
//        }
//    }

    public void setFieldCentricPower(double x, double y, double rotation){
        double angle = getAngle().getTheda(AngleUnit.RADIANS);

        double angleFromAuto = Robot.getInstance().getMecanumDrive().angleFromAuton;
        double xMod = x * Math.cos(angle - angleFromAuto) - y * Math.sin(angle - angleFromAuto);
        double yMod = x * Math.sin(angle - angleFromAuto) + y * Math.cos(angle - angleFromAuto);

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

    public double getSpeed(double xPosition, double yPosition, double rotation){
        double r = Math.hypot(xPosition, yPosition);
        double robotAngle = Math.atan2(yPosition, xPosition) - Math.PI / 4;
        double rightX = rotation;
        return (r * Math.cos(robotAngle) + rightX) * 1.414;
    }
}
