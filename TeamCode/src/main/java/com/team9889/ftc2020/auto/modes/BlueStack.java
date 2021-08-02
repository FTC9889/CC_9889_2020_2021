package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.BackIntakeDown;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Intake;
import com.team9889.lib.roadrunner.drive.DriveConstants;
import com.team9889.lib.roadrunner.drive.RoadRunner;

/**
 * Created by Eric on 7/29/2021.
 */
@Autonomous(group = "Blue", name = "Blue Stack", preselectTeleOp = "Teleop")
public class BlueStack extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        Robot.localizer.setPoseEstimate(new Pose2d(-63, 58, Math.toRadians(0)));
        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;

        switch (box) {
            case CLOSE:
                Robot.getFlyWheel().wantedMode = FlyWheel.Mode.AUTO;

                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, 58, Math.toRadians(0)))
                        .splineTo(new Vector2d(-20, 60), Math.toRadians(-13))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(5, 500, telemetry, 0, true));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-5, 60), Math.toRadians(45))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));
                ThreadAction(new BackIntakeDown());

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(-20, 60), Math.toRadians(-180))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new Wait(10000));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, 39), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;

            case MIDDLE:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, 58, Math.toRadians(0)))
                        .splineTo(new Vector2d(30, 60), Math.toRadians(0))
                        .splineTo(new Vector2d(33, 56), Math.toRadians(-90))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(700));
                ThreadAction(new BackIntakeDown());

                Robot.getFlyWheel().wantedMode = FlyWheel.Mode.AUTO;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(30, 60), Math.toRadians(-180))
                        .splineTo(new Vector2d(0, 60), Math.toRadians(-180))
                        .splineTo(new Vector2d(-10, 39), Math.toRadians(-180))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;
                runAction(new ShootRings(5, 500, telemetry));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(-20, 39), Math.toRadians(-180))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(3, 150, telemetry, 0, true));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, 60), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;

            case FAR:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, 58, Math.toRadians(0)))
                        .splineTo(new Vector2d(30, 63), Math.toRadians(0))
                        .splineTo(new Vector2d(45, 63), Math.toRadians(45))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));
                ThreadAction(new BackIntakeDown());

                Robot.getFlyWheel().wantedMode = FlyWheel.Mode.AUTO;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(10, 61), Math.toRadians(-180))
                        .splineTo(new Vector2d(-5, 39), Math.toRadians(-180))
                        .splineTo(new Vector2d(-15, 39), Math.toRadians(-180))
                        .build();
                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-5, 39), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(5, 500, telemetry));
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .lineToSplineHeading(new Pose2d(-25, 39, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-15, 39), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(3, 500, telemetry, 0, false));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .lineToSplineHeading(new Pose2d(-40, 39, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-15, 39), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(3, 500, telemetry, 0, true));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, 60), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;
        }

        Robot.getIntake().backIntakeOn = false;
        Robot.getIntake().passThroughIntakeOn = false;

        Robot.getIntake().currentArmPos = Intake.ArmPositions.HALF;
        runAction(new Wait(500));
        Robot.getIntake().currentArmPos = Intake.ArmPositions.UP;
        runAction(new Wait(500));
    }

    @Override
    public StartPosition side() {
        return StartPosition.BLUELEFT;
    }
}

