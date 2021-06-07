package com.team9889.lib.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.lib.roadrunner.drive.RoadRunner;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunner drive = new RoadRunner(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(60, 30), 0)
                .splineTo(new Vector2d(100, 0), 0)
//                .splineTo(new Vector2d(0, 0), 0)
                .build();

        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(80, -20), 0)
                .splineTo(new Vector2d(50, 60), 0)
//                .splineTo(new Vector2d(0, 0), 0)
                .build();

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}