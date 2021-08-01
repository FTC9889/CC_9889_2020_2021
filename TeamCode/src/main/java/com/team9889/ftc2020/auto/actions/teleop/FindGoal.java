package com.team9889.ftc2020.auto.actions.teleop;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 7/24/2021.
 */
public class FindGoal extends Action {
    ElapsedTime time = new ElapsedTime();

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void update() {
        if (Robot.getInstance().getFlyWheel().shooting || Robot.getInstance().getMecanumDrive().resetPos) {
            Robot.getInstance().getCamera().scanForGoal.findGoal();

            Pose2d updatedPos = Robot.getInstance().getCamera().getRobotPos();
            if (updatedPos.getX() < 900 && Math.abs(Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES)) < 40
                    && Robot.getInstance().getFlyWheel().shooting || (updatedPos.getX() < 900 && Robot.getInstance().getMecanumDrive().resetPos)) {
                Robot.getInstance().rr.setPoseEstimate(updatedPos);

                Robot.getInstance().rr.getLocalizer().update();

                Robot.getInstance().getCamera().resetPos = true;
            }
        }

//        Robot.getInstance().rr.getLocalizer().setPoseEstimate(new Pose2d(Robot.getInstance().rr.getLocalizer().getPoseEstimate().getX(), Robot.getInstance().rr.getLocalizer().getPoseEstimate().getY(),
//                Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES) - Math.toRadians(Robot.getInstance().getMecanumDrive().angleFromAuton)));

        Log.i("AI Loop Time", "" + time.milliseconds());
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
}
