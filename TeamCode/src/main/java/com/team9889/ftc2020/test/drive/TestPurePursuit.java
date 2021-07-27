//package com.team9889.ftc2020.test.drive;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.team9889.ftc2020.auto.AutoModeBase;
//import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
//import com.team9889.lib.control.Path;
//
//import java.util.ArrayList;
//
///**
// * Created by Eric on 8/24/2020.
// */
//
//@Disabled
//@Autonomous
//public class TestPurePursuit extends AutoModeBase {
//
//    @Override
//    public void run(StartPosition startPosition, Boxes box) {
//        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
//            add(new Path(new Pose2d(40, 0, 0),
//                    new Pose2d(1, 1, 2), 8, 1));
//            add(new Path(new Pose2d(40.1, -20, 0),
//                    new Pose2d(1, 1, 2), 8, 1));
//            add(new Path(new Pose2d(0, -40, 0),
//                    new Pose2d(1, 1, 2), 8, 1));
//            add(new Path(new Pose2d(50, -60, 20),
//                    new Pose2d(1, 1, 2), 8, 1));
//        }}));
//    }
//
//    @Override
//    public StartPosition side() {
//        return StartPosition.REDLEFT;
//    }
//}
