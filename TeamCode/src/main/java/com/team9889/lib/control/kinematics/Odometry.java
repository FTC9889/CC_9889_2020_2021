package com.team9889.lib.control.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.team9889.ftc2020.Constants;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.hardware.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

public class Odometry {
    //Odometry wheels
    private Motor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance = 9;
    private double horizontalEncoderTickPerDegreeOffset = .75;

//    private double verticalLeftEncoderPositionMultiplier = 1;
//    private double verticalRightEncoderPositionMultiplier = 1;
//    private double normalEncoderPositionMultiplier = 1;


    private double verticalLeftEncoderPositionMultiplier = 1.6666666666667;
    private double verticalRightEncoderPositionMultiplier = 1.6666666666667;
    private double normalEncoderPositionMultiplier = 1.6666666666667;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     */
    public Odometry(Motor verticalEncoderLeft, Motor verticalEncoderRight, Motor horizontalEncoder){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
    }

    double lastAngle;

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void update(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
//        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO);
        changeInRobotOrientation = Robot.getInstance().getMecanumDrive().getAngle().getTheda(AngleUnit.RADIANS) - lastAngle;

//        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
        robotOrientationRadians = (Robot.getInstance().getMecanumDrive().getAngle().getTheda(AngleUnit.RADIANS));
        lastAngle = robotOrientationRadians;

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getPosition() * normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
//        double horizontalChange = rawHorizontalChange;

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    public double returnLastAngle(){ return Math.toDegrees(lastAngle); }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition * Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return robotOrientationRadians; }

    public Pose2d returnPose () {
        return new Pose2d(returnXCoordinate(), returnYCoordinate(), returnOrientation());
    }

    public Point returnPoint () {
        return new Point(returnXCoordinate(), returnYCoordinate());
    }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }
}
