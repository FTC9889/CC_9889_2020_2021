package com.team9889.ftc2020;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class to store constants
 * Created by joshua9889 on 4/10/2017.
 */

public class Constants {

    //VuMark Licence Key
    public final static String kVuforiaLicenceKey = "AUUHzRL/////AAABmaGWp2jobkGOsFjHcltHEso2M1NH" +
            "Ko/nBDlUfwXKZBwui9l88f5YAT31+8u5yoy3IEJ1uez7rZrIyLdyR4cCMC+a6I7X/EzkR1F094ZegAsXdg9n" +
            "ht01zju+nxxi1+rabsSepS+TZfAa14/0rRidvX6jqjlpncDf8uGiP75f38cW6W4uFRPrLdufA8jMqODGux9d" +
            "w7VkFMqB+DQuNk8n1pvJP87FFo99kr653cjMwO4TYbztNmUYaQUXjHHNhOFxHufN42r7YcCErvX90n/gIvs4" +
            "wrvffXGyU/xkmSaTJzrgiy8R+ZJx2T0JcEJ0m1UUEoF2BmW4ONAVv/TkG9ESGf6iAmx66vrVm3tk6+YY+1q1";

    public final static String kRevHubMaster = "C";
    public final static String kRevHubSlave = "E";

    public final static String kWebcam = "Webcam";

    public static int side = Color.RED;
    public static Pose2d pose;

    /*---------------------
    |                     |
    |     Drivetrain!     |
    |                     |
    ---------------------*/

    //Settings for Drive class
    public static class DriveConstants {
        public final static String kLeftDriveMasterId = "lf";
        public final static String kRightDriveMasterId = "rf";
        public final static String kLeftDriveSlaveId = "lb";
        public final static String kRightDriveSlaveId = "rb";

        public final static double WheelbaseWidth = 14.5;
        public final static double WheelDiameter = 3.77953;

        /**
         * ticks to inch
         * (Wheel Diameter * PI) / Counts Per Rotation
         */
        public final static double ENCODER_TO_DISTANCE_RATIO = (WheelDiameter * Math.PI) / 537.6;
        public final static double AngleToInchRatio = (Math.PI / 180.) * (WheelbaseWidth / 2);
        public final static double InchToTick = 537.6 / (Math.PI * 4.1);
    }

    public static class OdometryConstants {

//        private static double WheelDiameter = 38.45 / 25.4;  // https://www.rotacaster.com.au/shop/35mm-rotacaster-wheels/index
        private static double WheelDiameter = 35 / 25.4;  // https://www.rotacaster.com.au/shop/35mm-rotacaster-wheels/index
        public final static double ENCODER_TO_DISTANCE_RATIO = ((WheelDiameter * Math.PI) * ((((double) 40) / ((double) 24)))) / 1440; // https://www.rotacaster.com.au/shop/35mm-rotacaster-wheels/index  Step 4
//        0.005007837
    }

   /*---------------------
    |                     |
    |       Intake        |
    |                     |
    ---------------------*/

    //Settings for Intake
    public static class IntakeConstants {
        public final static String kFrontIntakeMotorId = "fi";
        public final static String kBackIntakeMotorId = "bi";
        public final static String kPassThroughId = "pass";
        public final static String kRingDetector = "frontring";
        public final static String kLeftDist = "leftdist";
        public final static String kRightDist = "rightdist";
        public final static String kLeftArm = "leftarm";
        public final static String kRightArm = "rightarm";
    }

    /*---------------------
    |                     |
    |       Lift!         |
    |                     |
    ---------------------*/

    //Settings for Lift
    public static class ShooterConstants {
        public final static String kFlyWheel = "fw";
        public final static String kFWArm = "fwarm";
        public final static String kFWLock = "lock";
        public final static String kFWFlap = "flap";
        public final static String kHopperDist = "hopperdetector";
    }

    /*---------------------
    |                     |
    |    Wobble Goal!     |
    |                     |
    ---------------------*/

    //Settings for Lift
    public static class WobbleGoalConstants {
        public final static String kWGGrabber = "wggrabber";
        public final static String kWGLeft = "wgleft";
        public final static String kWGRight = "wgright";
        public final static String kAutoWG = "autowg";
    }

    /*---------------------
    |                     |
    |        Camera!      |
    |                     |
    ---------------------*/

    //Settings for Camera
    public static class CameraConstants {
        public final static String kCameraXId = "camerax";
        public final static String kCameraYId = "cameray";
    }
}
