package com.team9889.ftc2020.auto.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 11/22/2019.
 */
public class Intake extends Action {

    private ElapsedTime timer = new ElapsedTime();

    private boolean intake = true;
    private boolean fullSpeed = false;

    public Intake() {
        this.intake = true;
        this.fullSpeed = false;
    }

    public Intake(boolean intake) {
        this.intake = intake;
        this.fullSpeed = false;
    }

    public Intake(boolean intake, boolean fullSpeed) {
        this.intake = intake;
        this.fullSpeed = fullSpeed;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        if (intake) {
            if (timer.milliseconds() > 300) {
                Robot.getInstance().getIntake().SetFrontIntakePower(0);
                intake = false;
                timer.reset();
            }
        } else {
            if (timer.milliseconds() > 100) {
                Robot.getInstance().getIntake().SetFrontIntakePower(1);
                intake = true;
                timer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {}
}
