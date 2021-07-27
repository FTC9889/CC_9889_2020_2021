package com.team9889.ftc2020.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.lib.control.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by joshua9889 on 12/24/2019.
 */
@Autonomous(group = "Test")
@Disabled
public class OdometryWheelTest extends AutoModeBase {
    @Override
    public void run(StartPosition startPosition, Boxes box) {
        StartPosition startPosition_ = StartPosition.BLUELEFT;
        List<Path> pose = new ArrayList<>();
//        Robot.getMecanumDrive().setCurrentPose(new Pose2d());

//        while (opModeIsActive()){
//            Robot.update();
//
//            telemetry.addData("Left Odometry", -Robot.leftLift.getPosition());
//            telemetry.addData("Right Odometry", -Robot.intakeLeft.getPosition());
//            telemetry.addData("Side Odometry", Robot.intakeRight.getPosition());
//
//            telemetry.addData("Left Odometry", Robot.getMecanumDrive().Left_OdometryPosition());
//            telemetry.addData("Right Odometry", Robot.getMecanumDrive().Right_OdometryPosition());
//            telemetry.addData("Side Odometry", Robot.getMecanumDrive().Y_OdometryPosition());
//
//            telemetry.update();
//        }

//        pose.add(new FollowPath(new Pose2d(20, 0, 0), new Pose2d(2, 2, 3), 4, 1));
//        pose.add(new FollowPath(new Pose2d(20, 0, 0), new Pose2d(2, 2, 3), 4, 1));
//        runAction(new DriveFollowPath(pose));

        while (opModeIsActive()) {
//            telemetry.addData("Pose", Robot.getMecanumDrive().getCurrentPose());
            Robot.outputToTelemetry(telemetry);
            telemetry.update();
        }
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDLEFT;
    }
}
