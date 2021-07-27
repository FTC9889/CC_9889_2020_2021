package com.team9889.ftc2020.auto.actions.flywheel;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 4/9/2021.
 */
public class OneStack extends Action {
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();

    boolean extend = false;

    @Override
    public void start() {
        shootTimer.reset();
        totalTimer.reset();

        Robot.getInstance().fwLock.setPosition(0.4);

        Robot.getInstance().getIntake().SetBackIntakePower(1);
        Robot.getInstance().getIntake().SetPassThroughPower(1);

        Robot.getInstance().getCamera().setGoalCamPos();
    }

    @Override
    public void update() {
        if (shootTimer.milliseconds() > 380 && shootTimer.milliseconds() < 450) {
//            double dist = 37.852 * Math.exp(0.0192 * Robot.getInstance().getCamera().scanForGoal.getPointInPixels().y);
//            double rpm = (3.06 * dist) + 1041;

            Robot.getInstance().getFlyWheel().setRPM(1410);
        }

        if (totalTimer.milliseconds() > 2000) {
            Robot.getInstance().update();
            if (shootTimer.milliseconds() > 500) {
                if (extend) {
                    Robot.getInstance().fwArm.setPosition(0.5);
                    extend = false;
                } else {
                    Robot.getInstance().fwArm.setPosition(.65);
                    extend = true;
                }

                shootTimer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return totalTimer.milliseconds() >= 5000;
    }

    @Override
    public void done() {
//        Robot.getInstance().getFlyWheel().setMode(FlyWheel.Mode.OFF);
        Robot.getInstance().getIntake().SetBackIntakePower(0);
        Robot.getInstance().getIntake().SetPassThroughPower(0);
        Robot.getInstance().getFlyWheel().done = true;
        Robot.getInstance().fwArm.setPosition(0.5);
        Robot.getInstance().fwLock.setPosition(1);
    }
}
