package com.team9889.ftc2020.auto.actions.intake;

import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.subsystems.Robot;

/**
 * Created by Eric on 11/22/2019.
 */
public class IntakeStop extends Action {
    @Override
    public void start() {
        Robot.getInstance().getIntake().SetFrontIntakePower(0);
        Robot.getInstance().getIntake().SetBackIntakePower(0);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
