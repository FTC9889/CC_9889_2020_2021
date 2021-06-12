package com.team9889.ftc2020.auto.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.team9889.ftc2020.auto.AutoModeBase;
import com.team9889.ftc2020.auto.actions.utl.Wait;

/**
 * Created by Eric on 6/11/2021.
 */

@Autonomous(group = "Red", preselectTeleOp = "Teleop")
public class RedPark extends AutoModeBase {

    @Override
    public void run(StartPosition startPosition, Boxes box) {
        runAction(new Wait(timeToWait));

        Robot.getMecanumDrive().setPower(0, 1, 0);
        runAction(new Wait(1500));
        Robot.getMecanumDrive().setPower(0, 0, 0);
    }

    @Override
    public StartPosition side() {
        return StartPosition.REDRIGHT;
    }
}
