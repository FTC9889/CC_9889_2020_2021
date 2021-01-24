package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.drive.MecanumDriveSimpleAction;
import com.team9889.ftc2020.auto.actions.flywheel.RunFlyWheel;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGDown;
import com.team9889.ftc2020.auto.actions.wobblegoal.WGUp;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.PurePursuit;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Eric on 11/26/2019.
 */

@Autonomous
public class RedAuto extends AutoModeBase {
    ArrayList<Path> pose = new ArrayList<>();

    @Override
    public void run(Side side, Boxes box) {
//        pose.add(new Path(new Pose2d(0.1, 30, 0), new Pose2d(2, 2, 3), 8, 1));
//        pose.add(new Path(new Pose2d(0, 30, 90), new Pose2d(2, 2, 3), 8, 1));
//        pose.add(new Path(new Pose2d(0.1, 0, 90), new Pose2d(2, 2, 3), 8, 1));
//        pose.add(new Path(new Pose2d(0, 0, 0), new Pose2d(2, 2, 3), 8, 1));

//        runAction(new DrivePurePursuit(pose));
//        pose.clear();


        switch (box) {
            case CLOSE:
                pose.add(new Path(new Pose2d(0.1, 72, 0), new Pose2d(2, 2, 3), 8, 1));
                pose.add(new Path(new Pose2d(-7, 78, 0), new Pose2d(2, 2, 3), 8, 1));
                break;
            case MIDDLE:
                pose.add(new Path(new Pose2d(0.1, 80, 0), new Pose2d(2, 2, 3), 8, 1));
                pose.add(new Path(new Pose2d(24, 100, 0), new Pose2d(2, 2, 3), 8, 1));
                break;
            case FAR:
                pose.add(new Path(new Pose2d(0.1, 80, 0), new Pose2d(2, 2, 3), 8, 1));
                pose.add(new Path(new Pose2d(-5, 120, 0), new Pose2d(2, 2, 3), 8, 1));
                break;
        }

        runAction(new DrivePurePursuit(pose));
        pose.clear();

        runAction(new WGDown());

//        ThreadAction(new RunFlyWheel());

        pose.add(new Path(new Pose2d(6, 62, 0), new Pose2d(2, 2, 2), 8, 1));
        runAction(new DrivePurePursuit(pose));
        pose.clear();

        runAction(new ShootRings(3));
        runAction(new Wait(500));

        pose.add(new Path(new Pose2d(26, 58, 0), new Pose2d(2, 2, 3), 8, 1));
        pose.add(new Path(new Pose2d(24, 40, 0), new Pose2d(2, 2, 3), 8, .5));
        pose.add(new Path(new Pose2d(20, 35, 0), new Pose2d(2, 2, 3), 8, .5));
        runAction(new DrivePurePursuit(pose));
        pose.clear();

        pose.add(new Path(new Pose2d(20.1, 24, 0), new Pose2d(2, 2, 3), 8, .3));
        runAction(new DrivePurePursuit(pose));
        pose.clear();

        runAction(new PickUpWG());

        pose.add(new Path(new Pose2d(24, 58, -150), new Pose2d(2, 2, 3), 8, 1));
        switch (box) {
            case CLOSE:
                pose.add(new Path(new Pose2d(0, 58, -150), new Pose2d(2, 2, 3), 8, 1));
                break;
            case MIDDLE:
                pose.add(new Path(new Pose2d(28, 78, -150), new Pose2d(2, 2, 3), 8, 1));
                break;
            case FAR:
                pose.add(new Path(new Pose2d(5, 115, -150), new Pose2d(2, 2, 3), 8, 1));
                break;
        }

        runAction(new DrivePurePursuit(pose));
        pose.clear();

        runAction(new PutDownWG());

        runAction(new Wait(500));
        ThreadAction(new WGUp());

        Robot.getIntake().Intake();
        runAction(new Wait(250));
        Robot.getIntake().Stop();

        if (box == Boxes.CLOSE) {
            pose.add(new Path(new Pose2d(24, 65, -150), new Pose2d(2, 2, 3), 8, 1));
            pose.add(new Path(new Pose2d(24.1, 72, -90), new Pose2d(2, 2, 3), 8, 1));
            runAction(new DrivePurePursuit(pose));
            pose.clear();
        } else if (box == Boxes.MIDDLE) {
            pose.add(new Path(new Pose2d(25, 65, -150), new Pose2d(2, 2, 3), 8, 1));
            pose.add(new Path(new Pose2d(25, 65, -90), new Pose2d(2, 2, 3), 8, 1));
            runAction(new DrivePurePursuit(pose));
            pose.clear();
        } else {
            pose.add(new Path(new Pose2d(10, 72, -90), new Pose2d(2, 2, 3), 8, 1));
            runAction(new DrivePurePursuit(pose));
            pose.clear();
        }
    }
}
