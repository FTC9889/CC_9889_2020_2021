package com.team9889.ftc2019.auto.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2019.auto.AutoModeBase;
import com.team9889.ftc2019.auto.actions.drive.MecanumDriveSimpleAction;
import com.team9889.ftc2019.auto.actions.flywheel.ShootRings;
import com.team9889.ftc2019.auto.actions.utl.Wait;

/**
 * Created by Eric on 11/26/2019.
 */

@Autonomous
public class RedAuto extends AutoModeBase {
    @Override
    public void run(Side side, AutoModeBase.SkyStonePosition stonePosition) {
//        runAction(new ShootRings(3));
        runAction(new MecanumDriveSimpleAction(120, 0));
        runAction(new Wait(2000));
    }
}
