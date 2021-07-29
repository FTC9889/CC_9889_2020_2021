package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShots;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.lib.roadrunner.drive.DriveConstants;
import com.team9889.lib.roadrunner.drive.RoadRunner;

/**
 * Created by Eric on 6/4/2021.
 */

@Autonomous(group = "Red", name = "Full Left Red", preselectTeleOp = "Teleop")
public class FullLeftRed extends AutoModeBase {
    Trajectory traj;

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT1;
        runAction(new Wait(500));

//        Robot.getCamera().setPS1CamPos();
//        Robot.getCamera().setScanForGoal();

        Robot.localizer.setPoseEstimate(new Pose2d(-63, -17.5, Math.toRadians(0)));

        if (box == Boxes.FAR) {
            Robot.leftArm.setPosition(0.5);
            Robot.rightArm.setPosition(0.5);
        }

        traj = Robot.rr.trajectoryBuilder(new Pose2d(-63, -17.5, Math.toRadians(0)))
                .splineTo(new Vector2d(-10, -15), Math.toRadians(0))
                .build();
        Robot.rr.followTrajectory(traj);

        runAction(new PowerShots(true));
//        runAction(new PowerShots());

        if (box == Boxes.FAR) {
            Robot.leftArm.setPosition(0);
            Robot.rightArm.setPosition(1);
        }

        Robot.fwLock.setPosition(.4);

        switch (box) {
            case CLOSE:
                Robot.wgLeft.setPosition(.5);
                Robot.wgRight.setPosition(.5);

                traj = Robot.rr.trajectoryBuilder(traj.end())
//                        .splineTo(new Vector2d(-20, -30), Math.toRadians(0))
                        .splineTo(new Vector2d(10, -55), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);

                ThreadAction(new PutDownWG());

//                Robot.getIntake().SetFrontIntakePower(1);

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(-34, -50), Math.toRadians(180))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);
                Robot.getIntake().frontIntakeOn = true;
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                ThreadAction(new PickUpWG());
                runAction(new Wait(300));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-10, -50), Math.toRadians(0))
                        .splineTo(new Vector2d(25, -45), Math.toRadians(45))
                        .build();

                Robot.rr.followTrajectory(traj);
                runAction(new PutDownWG());

                Robot.wgLeft.setPosition(.6);
                Robot.wgRight.setPosition(.6);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(35, -55), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .lineTo(new Vector2d(54, -55))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(55, -50), Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(45))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.flyWheel.motor.setVelocity(1500);
                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(10, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(-10, -15), Math.toRadians(165))
                        .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(5, 0, telemetry, 1500, true));


                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
//                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);
                break;

            case MIDDLE:
                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
                        .splineTo(new Vector2d(25, -22), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.getIntake().frontIntakeOn = false;
                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                ThreadAction(new PutDownWG());
                Robot.flyWheel.motor.setVelocity(1450);
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                Robot.getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(15, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(-20, -35), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-25, -35), Math.toRadians(170),
                                RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);

                runAction(new ShootRings(5, 0, telemetry, 1500));
                runAction(new Wait(500));

                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);
                Robot.getIntake().backIntakeOn = false;

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineToConstantHeading(new Vector2d(-32, -50), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                ThreadAction(new PickUpWG());
                runAction(new Wait(300));

                Robot.getIntake().frontIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-10, -50), Math.toRadians(0))
                        .splineTo(new Vector2d(55, -50), Math.toRadians(0))
                        .splineTo(new Vector2d(56, -50), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                runAction(new PutDownWG());

                Robot.wgLeft.setPosition(.6);
                Robot.wgRight.setPosition(.6);
                runAction(new Wait(500));

                Robot.flyWheel.motor.setVelocity(1500);

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(56, -55), Math.toRadians(-90))
                        .build();
                Robot.rr.followTrajectory(traj);

                Robot.getIntake().frontIntakeOn = false;
                Robot.getIntake().backIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(58, -20), Math.toRadians(90))
                    .splineTo(new Vector2d(10, -20), Math.toRadians(180))
                    .splineTo(new Vector2d(-10, -20), Math.toRadians(165))
                    .build();
                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(5, 0, telemetry, 1500, true));

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, -15), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);
                break;

            case FAR:
                Robot.getIntake().frontIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(10, -20), Math.toRadians(0))
                        .splineTo(new Vector2d(58, -55), Math.toRadians(-45))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.getIntake().frontIntakeOn = false;
                Robot.autoWG.setPosition(.75);
                runAction(new Wait(200));

                ThreadAction(new PutDownWG());
                Robot.getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;
                Robot.getIntake().backIntakeOn = true;
                Robot.getIntake().passThroughIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(-5, -35), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-20, -35), Math.toRadians(180),
                                RoadRunner.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                traj = Robot.rr.trajectoryBuilder(traj.end(), false)
                        .splineToConstantHeading(new Vector2d(-15, -35), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(5, 0, telemetry, 1450, false));

                traj = Robot.rr.trajectoryBuilder(traj.end(), true)
                        .lineToSplineHeading(new Pose2d(-30, -35, Math.toRadians(0)),
                                RoadRunner.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                RoadRunner.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                Robot.rr.followTrajectory(traj);

                runAction(new ShootRings(10, 0, telemetry, 1470, true));

                Robot.getIntake().backIntakeOn = false;
                Robot.getIntake().frontIntakeOn = true;

                traj = Robot.rr.trajectoryBuilder(traj.end())
                        .splineToConstantHeading(new Vector2d(-32, -50), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                ThreadAction(new PickUpWG());
                runAction(new Wait(400));

                traj = Robot.rr.trajectoryBuilder(traj.end())
//                        .splineTo(new Vector2d(40, -30), Math.toRadians(0))
                        .splineTo(new Vector2d(55, -40), Math.toRadians(0))
                        .build();

                Robot.rr.followTrajectory(traj);

                Robot.rr.turn(Math.toRadians(100));
                runAction(new PutDownWG());
                ThreadAction(new PickUpWG(500));

                traj = Robot.rr.trajectoryBuilder(new Pose2d(traj.end().vec(), Math.toRadians(100)))
                        .splineTo(new Vector2d(55, -20), Math.toRadians(100))
                        .splineTo(new Vector2d(10, -20), Math.toRadians(180))
                        .build();

                Robot.rr.followTrajectory(traj);
                break;
        }
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDLEFT;
    }
}
