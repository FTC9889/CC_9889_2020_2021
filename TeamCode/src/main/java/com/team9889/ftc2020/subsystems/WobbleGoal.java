package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

public class WobbleGoal extends Subsystem{
    @Override
    public void init(boolean auto) {
        if (auto) {
            setFinger(0.6);
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

    public void down () {
        Robot.getInstance().wgLeft.setPosition(0.4);
        Robot.getInstance().wgRight.setPosition(0.4);
    }

    public void inBetween() {
        Robot.getInstance().wgLeft.setPosition(.2);
        Robot.getInstance().wgRight.setPosition(.2);
    }

    public void up () {
        Robot.getInstance().wgLeft.setPosition(0.8);
        Robot.getInstance().wgRight.setPosition(0.8);
    }

    public void open() {
        setFinger(0.6);
    }

    public void close() {
        setFinger(0.2);
    }

    private void setFinger(double position) {
        Robot.getInstance().wobbleGoalFinger.setPosition(position);
    }
}
