package com.team9889.ftc2020.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eric on 8/19/2019.
 */

public class Intake extends Subsystem {
    public int numSinceIntakeOn = 0;
    public int ringsIntaken = 0;
    boolean intakeReady = true;
    double currentPower = 0;
    public double current = 0;

    public boolean frontIntakeOn = false;
    public boolean backIntakeOn = false;
    public boolean passThroughIntakeOn = false;

    boolean auto = false;

    @Override
    public void init(boolean auto) {
        if (auto) {
            Robot.getInstance().leftArm.setPosition(0);
            Robot.getInstance().rightArm.setPosition(1);
        }

        this.auto = auto;
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Ring Intaken", ringsIntaken);
    }

    @Override
    public void update() {
        if (ringsIntaken >= 4) {
            frontIntakeOn = false;
            backIntakeOn = false;
        }

        if (auto) {
            if (frontIntakeOn) {
                SetFrontIntakePower(1);
            } else {
                SetFrontIntakePower(0);
            }
            if (backIntakeOn) {
                SetBackIntakePower(1);
            } else {
                SetBackIntakePower(0);
            }
            if (passThroughIntakeOn) {
                SetPassThroughPower(1);
            } else {
                SetPassThroughPower(0);
            }
        }


        if ( current > 5500 &&
                intakeReady && numSinceIntakeOn > 10) {
            intakeReady = false;
            ringsIntaken++;
        } else if (current <= 5500) {
            intakeReady = true;
        }

        numSinceIntakeOn++;
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

    public void SetPassThroughPower(double power){
        if (power != currentPower) {
            Robot.getInstance().passThrough.setPower(power);
            numSinceIntakeOn = 0;
            intakeReady = false;
            currentPower = power;
        }
    }
}