package com.team9889.ftc2020.auto.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.flywheel.OneStack;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShotsAuto;
import com.team9889.ftc2020.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2020.auto.actions.flywheel.Stack;
import com.team9889.ftc2020.auto.actions.intake.Outtake;
import com.team9889.ftc2020.auto.actions.teleop.PowerShots;
import com.team9889.ftc2020.auto.actions.utl.ParallelAction;
import com.team9889.ftc2020.auto.actions.utl.Wait;
import com.team9889.ftc2020.auto.actions.wobblegoal.PickUpWG;
import com.team9889.ftc2020.auto.actions.wobblegoal.PutDownWG;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
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
        Robot.getCamera().setPS1CamPosAuto();
        Robot.getFlyWheel().setMode(FlyWheel.Mode.POWERSHOT1);

//        ThreadAction(new PowerShotsAuto(3));

        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
            add(new Path(new Pose2d(49, 0, 0),
                    defaultTolerance, 8, 1));
            add(new Path(new Pose2d(50, -5, 0),
                    defaultTolerance, 8, 1));
//            add(new Path(new Pose2d(20, 0, 0),
//                    defaultTolerance, 8, 1));
//            add(new Path(new Pose2d(50, 0, 0),
//                    defaultTolerance, 8, 1));
//            add(new Path(new Pose2d(40, 0, 0),
//                    defaultTolerance, 8, 1));
                }}));

        Robot.getInstance().wgLeft.setPosition(0.7);
        Robot.getInstance().wgRight.setPosition(0.7);
        runAction(new PowerShotsAuto());
        Robot.getInstance().wgLeft.setPosition(0.8);
        Robot.getInstance().wgRight.setPosition(0.8);


        switch (box) {
            case CLOSE:
                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(62, -38, 0),
                            defaultTolerance, 8, 1));
                }}));
                break;

            case MIDDLE:
                Robot.getIntake().SetBackIntakePower(-.5);
                ThreadAction(new OneStack());
                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(60, -22, 0),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(40, -22, -1),
                            new Pose2d(2, 2, 3), 8, .8));
                }}));

                while (!Robot.getFlyWheel().done) {

                }

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(85, -10, 0),
                            defaultTolerance, 8, 1));
                }}));
                break;

            case FAR:
                Robot.getIntake().SetBackIntakePower(-.5);
                ThreadAction(new Stack());
                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(65, -20, 0),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(30, -20, -1),
                            new Pose2d(2, 2, 3), 8, .18));
                }}));

                while (!Robot.getFlyWheel().done) {

                }

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(110, -38, 0),
                            defaultTolerance, 8, 1));
                }}));
                break;
        }


        Robot.getIntake().SetFrontIntakePower(0);

        Robot.autoWG.setPosition(.75);
        runAction(new Wait(500));

        ThreadAction(new PutDownWG());

        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
            add(new Path(new Pose2d(40, -32, 0),
                    defaultTolerance, 20, 1));
        }}));
        runAction(new DrivePurePursuit(new ArrayList<Path>(){{
            add(new Path(new Pose2d(29.5, -33, 0),
                defaultTolerance, 8, .3));
        }}));

        runAction(new PickUpWG());

        switch (box) {
            case CLOSE:
                runAction(new DrivePurePursuit(new ArrayList<Path>() {{
                    add(new Path(new Pose2d(52, -30, -150),
                            defaultTolerance, 8, 1));
                }}));

                runAction(new PutDownWG());

                Robot.leftArm.setPosition(1);
                runAction(new Wait(500));

                Robot.getIntake().SetFrontIntakePower(1);
                Robot.getIntake().SetBackIntakePower(1);
                Robot.passThrough.setPower(1);
                Robot.leftArm.setPosition(0);

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(50, -10, -150),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(90, -10, 0),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(110, -20, 0),
                            defaultTolerance, 8, 1));
                }}));

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(119, 10, -60),
                            defaultTolerance, 8, 1));
                }}));

//                Robot.flyWheel.setPower(1300);

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(55, -20, 0),
                            defaultTolerance, 8, .8));
                }}));

                runAction(new ShootRings(2, 800, telemetry, 1360));
                break;

            case MIDDLE:
                runAction(new DrivePurePursuit(new ArrayList<Path>() {{
                    add(new Path(new Pose2d(75, -30, 150),
                            defaultTolerance, 8, 1));
                }}));

                runAction(new PutDownWG());

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(60, -20, 150),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(65, -20, 0),
                            defaultTolerance, 8, 1));
                }}));
                break;

            case FAR:
                runAction(new DrivePurePursuit(new ArrayList<Path>() {{
                    add(new Path(new Pose2d(90, -30, 0),
                            new Pose2d(5, 5, 5), 8, 1));
                }}));
                runAction(new DrivePurePursuit(new ArrayList<Path>() {{
                    add(new Path(new Pose2d(100, -25, -150),
                            defaultTolerance, 8, 1));
                }}));

                runAction(new PutDownWG());

                runAction(new DrivePurePursuit(new ArrayList<Path>(){{
                    add(new Path(new Pose2d(85, -30, -150),
                            defaultTolerance, 8, 1));
                    add(new Path(new Pose2d(70, -30, 0),
                            defaultTolerance, 8, 1));
                }}));
                break;
        }

        Robot.leftArm.setPosition(1);
        Robot.rightArm.setPosition(0);


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