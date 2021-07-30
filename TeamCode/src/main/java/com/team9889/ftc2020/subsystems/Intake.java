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
    public boolean passThroughIntakeOnPartial = false;
    public boolean outtake = false;

    boolean auto = false;

    public enum ArmPositions {
        DOWN, UP, HALF, NULL
    }
    public ArmPositions wantedArmPos = ArmPositions.NULL;
    public ArmPositions currentArmPos = ArmPositions.NULL;

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
        telemetry.addData("Current", current);
    }

    @Override
    public void update() {
//        if (ringsIntaken >= 4) {
//            frontIntakeOn = false;
//            backIntakeOn = false;
//        }

        if (frontIntakeOn) {
            if (outtake)
                SetFrontIntakePower(-1);
            else
                SetFrontIntakePower(1);
        } else {
            SetFrontIntakePower(0);
        }
        if (backIntakeOn) {
            if (outtake)
                SetBackIntakePower(-1);
            else
                SetBackIntakePower(1);
        } else {
            SetBackIntakePower(0);
        }

        if (passThroughIntakeOnPartial) {
            if (outtake)
                SetPassThroughPower(-.4);
            else
                SetPassThroughPower(.4);
        } else if (passThroughIntakeOn) {
            if (outtake)
                SetPassThroughPower(-1);
            else
                SetPassThroughPower(1);
        } else {
            SetPassThroughPower(0);
        }


        if (current > 5500 &&
                intakeReady && numSinceIntakeOn > 10) {
            intakeReady = false;
            ringsIntaken++;
        } else if (current <= 5500) {
            intakeReady = true;
        }

        numSinceIntakeOn++;

        if (currentArmPos != wantedArmPos) {
            switch (wantedArmPos) {
                case UP:
                    Robot.getInstance().leftArm.setPosition(0);
                    Robot.getInstance().rightArm.setPosition(1);
                    break;

                case HALF:
                    Robot.getInstance().leftArm.setPosition(0.5);
                    Robot.getInstance().rightArm.setPosition(0.5);
                    break;

                case DOWN:
                    Robot.getInstance().leftArm.setPosition(1);
                    Robot.getInstance().rightArm.setPosition(0);
                    break;

                case NULL:
                    break;
            }

            currentArmPos = wantedArmPos;
        }
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