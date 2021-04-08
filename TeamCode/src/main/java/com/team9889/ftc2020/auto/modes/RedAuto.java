package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShots;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.ParallelAction;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGDown;
import com.team9889.lib.control.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Eric on 11/26/2019.
 */

@Autonomous
public class RedAuto extends AutoModeBase {
    Pose2d defaultTolerance = new Pose2d(1, 1, 2);

    @Override
    public void run(Side side, Boxes box) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // I found https://www.geeksforgeeks.org/initialize-an-arraylist-in-java/
        // It seems easier to understand what is going on with the DrivePurePursuit

        // We also don't need a make a new list each time we want to add a new parallelaction, we
        // can add in right in with Arrays.asList
        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(1, 60, 0),
                            defaultTolerance, 8, 1));
                }}));

//        runAction(new PowerShots());

//        runAction(new Wait(500));
//
//        runAction(new ParallelAction(Arrays.asList(
//                new ShootRings(1, 1500, dashboardTelemetry),
//                new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(-14, 60, 0),
//                            defaultTolerance, 8, 1));
//                }})
//        )));
//
//        runAction(new Wait(500));
//
//        // We still need a arraylist for the pose for this section
//        ArrayList<Path> firstBoxPose = new ArrayList<>();
//        switch (box) {
//            case CLOSE:
//                firstBoxPose.add(new Path(new Pose2d(35, 75, 0),
//                        defaultTolerance, 8, 1));
//                break;
//            case MIDDLE:
//                firstBoxPose.add(new Path(new Pose2d(10, 95, 0),
//                        defaultTolerance, 8, 1));
//                break;
//            case FAR:
//                firstBoxPose.add(new Path(new Pose2d(35, 120, 0),
//                        defaultTolerance, 8, 1));
//                break;
//        }
//
//        runAction(new DrivePurePursuit(firstBoxPose));
//
//        // TODO: IDK but maybe we can just increase the timer in WGDown, and remove the Wait?
//        runAction(new WGDown());
//        runAction(new Wait(500));
//
//        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//            add(new Path(new Pose2d(30, 60, 0),
//                    defaultTolerance, 8, 1));
//            add(new Path(new Pose2d(30.1, 30, 0),
//                    defaultTolerance, 8, 1));
//        }}));
//
//        runAction(new PickUpWG());
//
//        switch (box) {
//            case CLOSE:
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(30, 65, 0),
//                            new Pose2d(3, 3, 3), 8, 1, 3000));
//                    add(new Path(new Pose2d(30.1, 65, -140), new
//                            Pose2d(10, 10, 3), 8, 1));
//                }}));
//
//
//                runAction(new PutDownWG());
//
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(20, 65, 0),
//                            new Pose2d(2, 2, 2), 8, 1));
//                }}));
//                break;
//            case MIDDLE:
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(0, 80, 0),
//                            new Pose2d(3, 3, 3), 8, 1, 3000));
//                    add(new Path(new Pose2d(0.1, 80, -140),
//                            new Pose2d(10, 10, 3), 8, 1));
//                }}));
//
//                runAction(new PutDownWG());
//
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(0, 70, 0),
//                            new Pose2d(2, 2, 2), 8, 1));
//                }}));
//                break;
//            case FAR:
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(30, 110, 0),
//                            new Pose2d(3, 3, 3), 8, 1, 3000));
//                    add(new Path(new Pose2d(30.1, 110, -140),
//                            new Pose2d(10, 10, 3), 8, 1));
//                }}));
//
//                runAction(new PutDownWG());
//
//                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//                    add(new Path(new Pose2d(25.1, 80, 0),
//                            new Pose2d(2, 2, 2), 8, 1));
//                }}));
//                break;
//        }

    }
}