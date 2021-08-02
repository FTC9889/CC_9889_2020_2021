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

/**
 * Created by Eric on 7/29/2021.
 */
@Autonomous(group = "Red", name = "Red Powershots", preselectTeleOp = "Teleop")
public class RedPowerShots extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        Robot.localizer.setPoseEstimate(new Pose2d(-63, -18, Math.toRadians(0)));

        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.AUTO;
        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;

        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -18, Math.toRadians(0)))
                .splineTo(new Vector2d(-10, -15), Math.toRadians(-15))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new ShootRings(6, 0, telemetry, 0, true));

        /*
        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT2;
        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;

        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -18, Math.toRadians(0)))
                .splineTo(new Vector2d(-10, -15), Math.toRadians(-15))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new ShootRings(1, 1500, telemetry, true, false));

//        Robot.rr.turn(Math.toRadians(7));
        traj = Robot.rr.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(-10, -14, Math.toRadians(6)))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new ShootRings(1, 500, telemetry, true, false));

//        Robot.rr.turn(Math.toRadians(8));
        traj = Robot.rr.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(-10, -13, Math.toRadians(13)))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new ShootRings(1, 500, telemetry, true, true));
         */

        switch (box) {
            case CLOSE:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-10, -13, Math.toRadians(15)))
                        .splineTo(new Vector2d(10, -13), Math.toRadians(0))
                        .splineTo(new Vector2d(30, -35), Math.toRadians(-90))
                        .splineTo(new Vector2d(25, -50), Math.toRadians(-130))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(30, -35), Math.toRadians(90))
                        .splineTo(new Vector2d(10, -13), Math.toRadians(180))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;

            case MIDDLE:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-10, -13, Math.toRadians(15)))
                        .splineTo(new Vector2d(20, -13), Math.toRadians(0))
                        .splineTo(new Vector2d(40, -22), Math.toRadians(-90))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(700));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(10, -13), Math.toRadians(180))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;

            case FAR:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-10, -13, Math.toRadians(15)))
                        .splineTo(new Vector2d(45, -13), Math.toRadians(0))
                        .splineTo(new Vector2d(58, -40), Math.toRadians(-90))
                        .splineTo(new Vector2d(58, -45), Math.toRadians(-45))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(45, -13), Math.toRadians(180))
                        .splineTo(new Vector2d(10, -13), Math.toRadians(180))
                        .build();
                Robot.rr.followTrajectory(traj);
                break;
        }

        ThreadAction(new BackIntakeDown());

        Robot.getIntake().wantedArmPos = Intake.ArmPositions.HALF;
        runAction(new Wait(500));
        Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;
        runAction(new Wait(2000));
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDLEFT;
    }
}

