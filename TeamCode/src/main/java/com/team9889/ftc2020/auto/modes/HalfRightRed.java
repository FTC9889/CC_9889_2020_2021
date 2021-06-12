package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGDown;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGUp;

/**
 * Created by Eric on 6/4/2021.
 */

@Autonomous(group = "Red", name = "Half Right Red", preselectTeleOp = "Teleop")
public class HalfRightRed extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        Robot.localizer.setPoseEstimate(new Pose2d(-63, -58, Math.toRadians(0)));

        runAction(new Wait(timeToWait));

        switch (box) {
            case CLOSE:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -58, Math.toRadians(0)))
                        .splineTo(new Vector2d(-8, -62), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.flyWheel.motor.setVelocity(1480);

                Robot.autoWG.setPosition(0.75);
                runAction(new Wait(500));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(15)))
                        .build();
                Robot.rr.followTrajectory(traj);

                ThreadAction(new WGDown());

                runAction(new ShootRings(10, 500, telemetry, 1480, true));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);

                ThreadAction(new WGUp());
                runAction(new Wait(200));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-10, -43, Math.toRadians(-50)))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.rightArm.setPosition(1);

                runAction(new Wait(500));
                break;

            case MIDDLE:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -58, Math.toRadians(0)))
                        .splineTo(new Vector2d(0, -58), Math.toRadians(0))
                        .splineTo(new Vector2d(20, -53), Math.toRadians(70))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.flyWheel.motor.setVelocity(1480);

                Robot.autoWG.setPosition(0.75);
                runAction(new Wait(500));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(15)))
                        .build();
                Robot.rr.followTrajectory(traj);

                ThreadAction(new WGDown());

                runAction(new ShootRings(10, 500, telemetry, 1480, true));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);

                ThreadAction(new WGUp());

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(5, -60, Math.toRadians(0)))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.rightArm.setPosition(1);
                Robot.leftArm.setPosition(0);

                runAction(new Wait(500));
                break;

            case FAR:
                traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -58, Math.toRadians(0)))
                        .splineTo(new Vector2d(40, -62), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.flyWheel.motor.setVelocity(1480);

                Robot.autoWG.setPosition(0.75);
                runAction(new Wait(500));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(15)))
                        .build();
                Robot.rr.followTrajectory(traj);

                ThreadAction(new WGDown());

                runAction(new ShootRings(10, 500, telemetry, 1480, true));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);

                ThreadAction(new WGUp());

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineToSplineHeading(new Pose2d(5, -60, Math.toRadians(0)))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.rightArm.setPosition(1);
                Robot.leftArm.setPosition(0);

                runAction(new Wait(500));
                break;
        }
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDRIGHT;
    }
}
