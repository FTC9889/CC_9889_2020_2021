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
import com.team9889.ftc2020.auto.actions.intake.Intake;
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
        telemetry = dashboard.getTelemetry();

        actions.add(new ShootRings(1, 3000, telemetry));

        pose.add(new Path(new Pose2d(1, 60, 0), new Pose2d(.5, .5, 1), 8, 1));

        actions.add(new DrivePurePursuit(pose));

        runAction(new ParallelAction(actions));
        pose.clear();
        actions.clear();


        runAction(new Wait(500));


        actions.add(new ShootRings(1, 1500, telemetry));

        pose.add(new Path(new Pose2d(-6, 60, 0), new Pose2d(.5, .5, 1), 8, .4));
        actions.add(new DrivePurePursuit(pose));

        runAction(new ParallelAction(actions));
        pose.clear();
        actions.clear();


        runAction(new Wait(500));


        actions.add(new ShootRings(1, 1500, telemetry));

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
                pose.add(new Path(new Pose2d(10, 100, 0), new Pose2d(1, 1, 2), 8, 1));
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
        Robot.getIntake().Outtake();
        runAction(new Wait(100));
        Robot.getIntake().Intake();
        runAction(new Wait(100));

        switch (box) {
            case CLOSE:
                break;
            case MIDDLE:
                Robot.getIntake().Intake();
                pose.add(new Path(new Pose2d(40, 60, 0), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(30, 40, 0), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(20, 40, 0), new Pose2d(1, 1, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
            case FAR:
                Robot.getIntake().SetIntakePower(.6);
                Robot.arm.setPosition(0);

                pose.add(new Path(new Pose2d(30, 40, 0), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(25, 40, 0), new Pose2d(1, 1, 2), 8, .5));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                ThreadAction(new Intake());

                pose.add(new Path(new Pose2d(15, 40, 0), new Pose2d(1, 1, 2), 8, .3));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
        }
        Robot.arm.setPosition(1);

        Robot.flyWheel.motor.setVelocity(0);
        Robot.getFlyWheel().wantedFWSpeed = 0;
        Robot.getFlyWheel().counter = 0;
        Robot.getFlyWheel().lastMotorPos = Robot.flyWheel.getPosition();
        Robot.getFlyWheel().pid.error_prior = 0;
        Robot.getFlyWheel().pid.first = true;

        pose.add(new Path(new Pose2d(30, 40, 0), new Pose2d(1, 1, 2), 8, 1));
        actions.add(new DrivePurePursuit(pose));
        runAction(new ParallelAction(actions));
        actions.clear();
        pose.clear();
        pose.add(new Path(new Pose2d(30.1, 28, 0), new Pose2d(1, 1, 2), 8, 1));
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

                pose.add(new Path(new Pose2d(10, 100, 90), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(35, 118, 125), new Pose2d(3, 3, 2), 8, 1));
//                pose.add(new Path(new Pose2d(0.1, 85, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                pose.add(new Path(new Pose2d(-20, 118, 40), new Pose2d(1, 1, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                actions.add(new ShootRings(8, 4000, telemetry, 1240));
//                pose.add(new Path(new Pose2d(-20, 55, 120), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(15, 55, 0), new Pose2d(1, 1, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                pose.add(new Path(new Pose2d(10, 75, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
            case MIDDLE:
//                pose.add(new Path(new Pose2d(0, 85, 90), new Pose2d(3, 3, 3), 8, 1, 3000));
//                pose.add(new Path(new Pose2d(0.1, 85, -140), new Pose2d(10, 10, 3), 8, 1));
//                actions.add(new DrivePurePursuit(pose));
//                runAction(new ParallelAction(actions));
//                actions.clear();
//                pose.clear();

                pose.add(new Path(new Pose2d(-20, 100, 90), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(30.1, 125, 130), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(45, 125, 130), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(-30, 120, 50), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(0, 85, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                runAction(new PutDownWG());

                actions.add(new ShootRings(4, 2500, telemetry, 1260));
                pose.add(new Path(new Pose2d(5, 60, 0), new Pose2d(1, 1, 2), 8, 1));
                pose.add(new Path(new Pose2d(15, 60, 0), new Pose2d(1, 1, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                pose.add(new Path(new Pose2d(0, 70, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
            case FAR:
                pose.add(new Path(new Pose2d(25, 110, -90), new Pose2d(3, 3, 3), 8, 1, 3000));
                pose.add(new Path(new Pose2d(25.1, 110, -140), new Pose2d(10, 10, 3), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();


                runAction(new PutDownWG());

                actions.add(new ShootRings(8, 3000, telemetry, 1240));
                pose.add(new Path(new Pose2d(15, 60, 0), new Pose2d(1, 1, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();

                pose.add(new Path(new Pose2d(25.1, 75, 0), new Pose2d(2, 2, 2), 8, 1));
                actions.add(new DrivePurePursuit(pose));
                runAction(new ParallelAction(actions));
                actions.clear();
                pose.clear();
                break;
        }
    }
}