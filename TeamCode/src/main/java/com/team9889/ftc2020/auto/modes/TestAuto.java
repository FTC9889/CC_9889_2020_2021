package com.team9889.ftc2020.auto.modes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.lib.roadrunner.drive.DriveConstants;
import com.team9889.lib.roadrunner.drive.RoadRunner;

/**
 * Created by Eric on 6/4/2021.
 */

@Autonomous
public class TestAuto extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(Side side, Boxes box) {
        Robot.getInstance().getCamera().setPS1CamPos();
        Robot.getInstance().getCamera().setScanForGoal();

        Robot.localizer.setPoseEstimate(new Pose2d(-63, -17.5, Math.toRadians(0)));

//        Robot.getCamera().setPS1CamPosAuto();
//        Robot.getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);
//
//        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -17.5, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(-10, -35, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(-30, -35, Math.toRadians(Robot.getCamera().getPosOfTarget().x * 30)))
//                .build();
//
//        Robot.rr.followTrajectory(traj);
//
//        Robot.fwFlap.setPosition(.57);
//        runAction(new ShootRings(1, 500, telemetry, 1300, false));
////        Robot.getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);
//
//        traj = Robot.rr.trajectoryBuilder(traj.end())
//                .lineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(8)))
//                .build();
//
//        Robot.rr.followTrajectory(traj);
//
//        Robot.fwFlap.setPosition(.55);
//        runAction(new ShootRings(1, 400, telemetry, 1300, false));
////        Robot.getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);
//
//        traj = Robot.rr.trajectoryBuilder(traj.end())
//                .lineToSplineHeading(new Pose2d(-30, -15, Math.toRadians(8)))
//                .build();
//
//        Robot.rr.followTrajectory(traj);
//
//        Robot.fwFlap.setPosition(.55);
//        runAction(new ShootRings(1, 400, telemetry, 1300, true));
        Robot.leftArm.setPosition(0.5);
        Robot.rightArm.setPosition(0.5);

        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -17.5, Math.toRadians(0)))
                .splineTo(new Vector2d(-50, -17), Math.toRadians(0))
                .build();
        Robot.rr.followTrajectory(traj);

        Robot.leftArm.setPosition(0);
        Robot.rightArm.setPosition(1);

        switch (box) {
            case CLOSE:
                break;

            case MIDDLE:
                break;

            case FAR:
                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
                        .splineTo(new Vector2d(52, -50), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                ThreadAction(new PutDownWG());

                Robot.getIntake().SetBackIntakePower(1);
                Robot.passThrough.setPower(1);
                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(15, -35), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-15, -35), Math.toRadians(180),
                                RoadRunner.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                ThreadAction(new ShootRings(50, 1000, telemetry, 1530));

//                traj = Robot.rr.trajectoryBuilder(traj.end())
//                        .lineToSplineHeading(new Pose2d(-15, -33, Math.toRadians(0)),
//                                RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .build();
//
//                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-25, -35, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-22, -35, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                runAction(new Wait(2000));

                Robot.getIntake().SetBackIntakePower(0);
                Robot.getIntake().SetFrontIntakePower(1);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineToConstantHeading(new Vector2d(-32, -50), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                ThreadAction(new PickUpWG());
                runAction(new Wait(300));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(45, -40), Math.toRadians(120))
                        .build();

                Robot.rr.followTrajectory(traj);
                runAction(new PutDownWG());

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(40, -30), Math.toRadians(120))
                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);
                break;
        }

        Log.i("Angle", "" + calculateAngle());
    }

    public double calculateAngle() {
        double x = com.team9889.ftc2020.subsystems.Robot.getInstance().rr.getLocalizer().getPoseEstimate().getX() + 63;
        double y = com.team9889.ftc2020.subsystems.Robot.getInstance().rr.getLocalizer().getPoseEstimate().getY() + 63;

//        x =

        double hypot = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

//        double hypot = Math.sqrt(Math.pow(-20, 2) + Math.pow(-30, 2));

        double angle = com.team9889.ftc2020.subsystems.Robot.getInstance().rr.getLocalizer().getPoseEstimate().getY() / hypot;
        angle = Math.asin(angle);

        return angle;
    }
}
