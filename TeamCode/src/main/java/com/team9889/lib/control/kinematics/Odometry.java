package com.team9889.lib.control.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.team9889.ftc2020.Constants;
import com.team9889.lib.hardware.Motor;

import org.opencv.core.Point;

public class Odometry {
    //Odometry wheels
    private Motor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance = 16.2;
    private double horizontalEncoderTickPerDegreeOffset = -5;

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     */
    public Odometry(Motor verticalEncoderLeft, Motor verticalEncoderRight, Motor horizontalEncoder, double COUNTS_PER_INCH){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;

        robotEncoderWheelDistance *= COUNTS_PER_INCH;
    }

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
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getPosition() * normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition / Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition / Constants.OdometryConstants.ENCODER_TO_DISTANCE_RATIO; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

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
