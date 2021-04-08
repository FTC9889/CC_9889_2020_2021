package com.team9889.ftc2020.test.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.lib.control.Path;

import java.util.ArrayList;

/**
 * Created by Eric on 8/24/2020.
 */

@Autonomous
public class TestPurePursuit extends AutoModeBase {

    @Override
    public void run(Side side, Boxes box) {
        ArrayList<Path> paths = new ArrayList<>();

        Side Side_ = Side.RED;

        paths.add(new Path(new Pose2d(150, 0, 0), new Pose2d(1, 1, 2), 100, .8));
//        paths.add(new Path(new Pose2d(20, 0, 0), new Pose2d(2, 3, 5), 6, .3));

        runAction(new DrivePurePursuit(paths));
    }
}
