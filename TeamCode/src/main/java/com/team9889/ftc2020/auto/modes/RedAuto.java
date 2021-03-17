package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.drive.MecanumDriveSimpleAction;
import com.team9889.ftc2020.auto.actions.flywheel.RunFlyWheel;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.utl.ParallelAction;
import com.team9889.ftc2020.auto.actions.utl.RobotUpdate;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGDown;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGUp;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.PurePursuit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Eric on 11/26/2019.
 */

@Autonomous
public class RedAuto extends AutoModeBase {
    ArrayList<Action> actions = new ArrayList<>();
    ArrayList<Path> pose = new ArrayList<>();

    @Override
    public void run(Side side, Boxes box) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        actions.add(new ShootRings(1, 3000, dashboardTelemetry));

        pose.add(new Path(new Pose2d(1, 60, 0), new Pose2d(.5, .5, 1), 8, 1));

        actions.add(new DrivePurePursuit(pose));

        runAction(new ParallelAction(actions));
        pose.clear();
        actions.clear();


        runAction(new Wait(500));


        actions.add(new ShootRings(1, 1500, dashboardTelemetry));

        pose.add(new Path(new Pose2d(-6, 60, 0), new Pose2d(.5, .5, 1), 8, .4));
        actions.add(new DrivePurePursuit(pose));

        runAction(new ParallelAction(actions));
        pose.clear();
        actions.clear();


        runAction(new Wait(500));


        actions.add(new ShootRings(1, 1500, dashboardTelemetry));

        pose.add(new Path(new Pose2d(-14, 60, 0), new Pose2d(.5, .5, 1), 8, 1));
        actions.add(new DrivePurePursuit(pose));

        runAction(new ParallelAction(actions));
        pose.clear();
        actions.clear();


        runAction(new Wait(500));
//        Robot.getFlyWheel().psPower = true;
//        Robot.getCamera().setPS1CamPos();
//
//        pose.add(new Path(new Pose2d(1, 60, 0), new Pose2d(1, 1, 1), 8, 1));
//
//        actions.add(new DrivePurePursuit(pose));
//
//        runAction(new ParallelAction(actions));
//        pose.clear();
//        actions.clear();
//
//        actions.add(new AimAndShoot());
//        actions.add(new ShootRings(1, 1000, dashboardTelemetry));
//
//        runAction(new ParallelAction(actions));
//        pose.clear();
//        actions.clear();
//
//        Robot.getCamera().setPS2CamPos();
//        runAction(new Wait(500));
//
//        actions.add(new AimAndShoot());
//        actions.add(new ShootRings(1, 1500, dashboardTelemetry));
//
////        pose.add(new Path(new Pose2d(-6, 60, 0), new Pose2d(1, 1, 1), 8, 1));
////        actions.add(new DrivePurePursuit(pose));
//
//        runAction(new ParallelAction(actions));
//        pose.clear();
//        actions.clear();
//
//
//        Robot.getCamera().setPS2CamPos();
//        runAction(new Wait(500));
//
//        actions.add(new AimAndShoot());
//        actions.add(new ShootRings(1, 1500, dashboardTelemetry));
//
////        pose.add(new Path(new Pose2d(-14, 60, 0), new Pose2d(1, 1, 1), 8, 1));
////        actions.add(new DrivePurePursuit(pose));
//
//        runAction(new ParallelAction(actions));
//        pose.clear();
//        actions.clear();
//
//
//        runAction(new Wait(500));
//
//        Robot.getFlyWheel().psPower = false;
//

        switch (box) {
            case CLOSE:
                pose.add(new Path(new Pose2d(35, 75, 0), new Pose2d(1, 1, 2), 8, 1));
                break;
            case MIDDLE:
                pose.add(new Path(new Pose2d(10, 95, 0), new Pose2d(1, 1, 2), 8, 1));
                break;
            case FAR:
                pose.add(new Path(new Pose2d(35, 120, 0), new Pose2d(1, 1, 2), 8, 1));
                break;
        }

        actions.add(new DrivePurePursuit(pose));
        runAction(new ParallelAction(actions));
        actions.clear();
        pose.clear();


        runAction(new WGDown());
        runAction(new Wait(500));

        pose.add(new Path(new Pose2d(30, 60, 0), new Pose2d(1, 1, 2), 8, 1));
        pose.add(new Path(new Pose2d(30.1, 30, 0), new Pose2d(1, 1, 2), 8, 1));
        actions.add(new DrivePurePursuit(pose));
        runAction(new ParallelAction(actions));
        actions.clear();
        pose.clear();


        runAction(new PickUpWG());


        switch (box) {
            case CLOSE:
                pose.add(new Path(new Pose2d(25, 58, 0), new Pose2d(3, 3, 3), 8, 1, 3000));
                pose.add(new Path(new Pose2d(30.1, 65, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();


                runAction(new PutDownWG());

                pose.add(new Path(new Pose2d(20, 65, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
            case MIDDLE:
                pose.add(new Path(new Pose2d(0, 80, 0), new Pose2d(3, 3, 3), 8, 1, 3000));
                pose.add(new Path(new Pose2d(0.1, 80, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();


                runAction(new PutDownWG());


                pose.add(new Path(new Pose2d(0, 70, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
            case FAR:
                pose.add(new Path(new Pose2d(30, 110, 0), new Pose2d(3, 3, 3), 8, 1, 3000));
                pose.add(new Path(new Pose2d(30.1, 110, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();


                runAction(new PutDownWG());


                pose.add(new Path(new Pose2d(25.1, 80, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
        }

    }
}