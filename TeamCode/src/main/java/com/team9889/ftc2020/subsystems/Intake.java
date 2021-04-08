package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 8/19/2019.
 */

public class Intake extends Subsystem {

    @Override
    public void init(boolean auto) {
        if (auto) {
            Robot.getInstance().leftArm.setPosition(1);
            Robot.getInstance().rightArm.setPosition(1);
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
        SetFrontIntakePower(0);
        SetBackIntakePower(0);
    }

    public void SetFrontIntakePower(double power){
        Robot.getInstance().frontIntake.setPower(power);
    }

    public void SetBackIntakePower(double power){
        Robot.getInstance().backIntake.setPower(power);
    }
}