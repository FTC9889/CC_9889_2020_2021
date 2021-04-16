package com.team9889.ftc2020.auto.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 11/22/2019.
 */
public class Outtake extends Action {

    private ElapsedTime timer = new ElapsedTime();

    private boolean outtake = true;
    private boolean fullSpeed = false;

    public Outtake() {
        this.outtake = true;
        this.fullSpeed = false;
    }

    public Outtake(boolean outtake) {
        this.outtake = outtake;
        this.fullSpeed = false;
    }

    public Outtake(boolean outtake, boolean fullSpeed) {
        this.outtake = outtake;
        this.fullSpeed = fullSpeed;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
//        if(timer.milliseconds() > 100 && timer.milliseconds() < 200) {
        if(outtake) {
            Robot.getInstance().getIntake().SetFrontIntakePower(-0.3);
        } else {
            Robot.getInstance().getIntake().SetFrontIntakePower(0);
        }
//        }
//        else if(timer.milliseconds() > 200) {
//            if (outtake) Robot.getInstance().getIntake().SetRollerPower(1);
//            else Robot.getInstance().getIntake().SetRollerPower(0);
//        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 500;
    }

    @Override
    public void done() {}
}
