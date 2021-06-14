package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShots;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;

/**
 * Created by Eric on 6/4/2021.
 */

@Autonomous(group = "Red", name = "Half Left Red (DV)", preselectTeleOp = "Teleop")
public class HalfLeftRed extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        Robot.flyWheel.motor.setVelocity(1290);

        Robot.getCamera().setPS1CamPos();
        Robot.getCamera().setScanForGoal();

        Robot.localizer.setPoseEstimate(new Pose2d(-63, -17.5, Math.toRadians(0)));

        if (box == Boxes.FAR) {
            Robot.leftArm.setPosition(0.5);
            Robot.rightArm.setPosition(0.5);
        }

        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -17.5, Math.toRadians(0)))
                .splineTo(new Vector2d(-20, -17), Math.toRadians(0))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new PowerShots(true));

        if (box == Boxes.FAR) {
            Robot.leftArm.setPosition(0);
            Robot.rightArm.setPosition(1);
        }

        switch (box) {
            case CLOSE:
                Robot.wgLeft.setPosition(.5);
                Robot.wgRight.setPosition(.5);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(0, -17), Math.toRadians(0))
                        .splineTo(new Vector2d(25, -50), Math.toRadians(-135))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);
                runAction(new Wait(500));
                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);
                runAction(new Wait(500));

                Robot.rr.turn(Math.toRadians(90));

                Robot.getIntake().frontIntakeOn = true;
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                Robot.getFlyWheel().setRPM(1500);

                traj = Robot.rr.trajectoryBuilder(traj.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                        .splineTo(new Vector2d(55, -55), Math.toRadians(90))
                        .splineTo(new Vector2d(60, -15), Math.toRadians(90))
                        .splineTo(new Vector2d(55, -10), Math.toRadians(180))
                        .splineTo(new Vector2d(50, -15), Math.toRadians(-90))
                        .splineTo(new Vector2d(50, -55), Math.toRadians(-90))
                        .splineTo(new Vector2d(45, -59), Math.toRadians(-180))
                        .splineTo(new Vector2d(40, -55), Math.toRadians(90))
                        .splineTo(new Vector2d(40, -15), Math.toRadians(90))
                        .splineTo(new Vector2d(35, -10), Math.toRadians(180))
                        .splineTo(new Vector2d(30, -15), Math.toRadians(-90))
                        .splineTo(new Vector2d(30, -25), Math.toRadians(-90))
                        .splineTo(new Vector2d(25, -30), Math.toRadians(-180))
                        .splineTo(new Vector2d(-15, -30), Math.toRadians(135))
                        .splineTo(new Vector2d(-4, -35), Math.toRadians(0))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.fwLock.setPosition(.4);
                runAction(new ShootRings(10, 0, telemetry, 1500, true));

                Robot.leftArm.setPosition(1);
                runAction(new Wait(500));
                break;

            case MIDDLE:
                Robot.wgLeft.setPosition(.5);
                Robot.wgRight.setPosition(.5);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(55, -10), Math.toRadians(0))
                        .splineTo(new Vector2d(55, -15), Math.toRadians(-90))
                        .splineTo(new Vector2d(50, -25), Math.toRadians(-135))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(500));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);
                runAction(new Wait(500));
                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);
                runAction(new Wait(500));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(60, -20), Math.toRadians(90))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.getIntake().frontIntakeOn = true;
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                Robot.getFlyWheel().setRPM(1500);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(60, -55), Math.toRadians(-90))
                        .splineTo(new Vector2d(55, -60), Math.toRadians(-180))
                        .splineTo(new Vector2d(-35, -60), Math.toRadians(-180))
                        .splineTo(new Vector2d(-40, -55), Math.toRadians(90))
                        .splineTo(new Vector2d(-40, -25), Math.toRadians(90))
                        .splineTo(new Vector2d(-5, -10), Math.toRadians(-20))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.fwLock.setPosition(.4);
                runAction(new ShootRings(10, 0, telemetry, 1500, true));

                Robot.leftArm.setPosition(1);
                runAction(new Wait(500));
                break;

            case FAR:
                Robot.wgLeft.setPosition(.5);
                Robot.wgRight.setPosition(.5);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(55, -10), Math.toRadians(0))
                        .splineTo(new Vector2d(60, -15), Math.toRadians(-90))
                        .splineTo(new Vector2d(60, -45), Math.toRadians(-90))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);
                runAction(new Wait(500));
                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);

//                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(60, -35), Math.toRadians(90))
//                        .build();
//                Robot.rr.followTrajectory(traj);

//                Robot.getIntake().frontIntakeOn = true;
//                Robot.getIntake().backIntakeOn = true;
//                Robot.getIntake().passThroughIntakeOn = true;

//                Robot.getFlyWheel().setRPM(1500);

//                traj = Robot.rr.trajectoryBuilder(traj.end())
//                        .splineTo(new Vector2d(50, -40), Math.toRadians(-180))
//                        .splineTo(new Vector2d(40, -45), Math.toRadians(-90))
//                        .splineTo(new Vector2d(37.5, -55), Math.toRadians(-180))
//                        .splineTo(new Vector2d(35, -50), Math.toRadians(90))
//                        .splineTo(new Vector2d(45, -15), Math.toRadians(90))
//                        .splineTo(new Vector2d(40, -10), Math.toRadians(180))
//                        .splineTo(new Vector2d(35, -15), Math.toRadians(-90))
//                        .splineTo(new Vector2d(25, -50), Math.toRadians(-90))
//                        .splineTo(new Vector2d(20, -55), Math.toRadians(-180))
//                        .splineTo(new Vector2d(-35, -60), Math.toRadians(-180))
//
//                        .splineTo(new Vector2d(-40, -45), Math.toRadians(90))
//                        .splineTo(new Vector2d(-40, -25), Math.toRadians(90))
//                        .splineTo(new Vector2d(-10, -10), Math.toRadians(-17))
//                        .build();
//                Robot.rr.followTrajectory(traj);
//
//                Robot.fwLock.setPosition(.4);
//                runAction(new ShootRings(6, 0, telemetry, 1500, true));
//
//                Robot.leftArm.setPosition(1);
//                traj = Robot.rr.trajectoryBuilder(traj.end())
//                        .lineTo(new Vector2d(0, -10))
//                        .build();
//                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(60, -15), Math.toRadians(-90))
                        .splineTo(new Vector2d(40, -10), Math.toRadians(180))
                        .splineTo(new Vector2d(10, -10), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                runAction(new Wait(500));
                break;
        }
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDLEFT;
    }
}
