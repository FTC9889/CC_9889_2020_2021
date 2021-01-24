package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class WobbleGoal extends Subsystem{
    @Override
    public void init(boolean auto) {
        if (auto) {
            Robot.getInstance().wgGrabber.setPosition(.65);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    public void setWGDown () {
        Robot.getInstance().wgLeft.setPosition(0.4);
        Robot.getInstance().wgRight.setPosition(0.4);
    }

    public void setWGUp () {
        Robot.getInstance().wgLeft.setPosition(0.8);
        Robot.getInstance().wgRight.setPosition(0.8);
    }
}
