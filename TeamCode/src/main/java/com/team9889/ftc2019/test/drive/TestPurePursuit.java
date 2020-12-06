package com.team9889.ftc2019.test.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2019.Team9889Linear;
import com.team9889.ftc2019.auto.AutoModeBase;
import com.team9889.ftc2019.auto.actions.drive.DrivePurePursuit;
import com.team9889.lib.Path;
import com.team9889.lib.PurePursuit;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Eric on 8/24/2020.
 */

@Autonomous
public class TestPurePursuit extends AutoModeBase {

    @Override
    public void run(Side side) {
        ArrayList<Path> paths = new ArrayList<>();

        Side Side_ = Side.RED;
        Robot.redAuto = true;

        paths.add(new Path(new Pose2d(0, 48, 0), new Pose2d(2, 2, 3), 6, .5));
        paths.add(new Path(new Pose2d(48, 48, 90), new Pose2d(2, 2, 3), 6, 1));
        paths.add(new Path(new Pose2d(48, 0, 0), new Pose2d(2, 2, 3), 6, .5));
        paths.add(new Path(new Pose2d(0, 0, 0), new Pose2d(2, 2, 3), 6, 1));

        runAction(new DrivePurePursuit(paths));
    }
}
