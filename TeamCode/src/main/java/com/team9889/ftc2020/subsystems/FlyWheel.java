package com.team9889.ftc2020.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by joshua9889 on 3/28/2018.
 */

@Config
public class FlyWheel extends Subsystem{
    public enum Mode {
        OFF, POWERSHOT1, POWERSHOT2, POWERSHOT3, POWERSHOTAUTO1, POWERSHOTAUTO2, POWERSHOTAUTO3, DEFAULT
    }
    public Mode currentMode = Mode.OFF;
    public Mode wantedMode = Mode.OFF;

    double time = 200;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean extend = false;

    public static double P = 150, I = 0, D = 20, F = 0.3;
    public PIDF pid = new PIDF(150, 0, 20, 0.3);

    public boolean done = false;

    public boolean autoPower = false;

    public enum RampPositions {
        UP, DOWN, PS, NULL
    }
    public RampPositions currentRampPos = RampPositions.NULL;
    public RampPositions wantedRampPos = RampPositions.NULL;

    @Override
    public void init(boolean auto) {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        if (auto) {
            setRampUp();
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("Fly Wheel Speed", getRPM());
    }

    @Override
    public void update() {
        if (currentRampPos != wantedRampPos) {
            switch (wantedRampPos) {
                case UP:
                    Robot.getInstance().fwFlap.setPosition(0.7);
                    break;

                case DOWN:
                    Robot.getInstance().fwFlap.setPosition(.4);
                    break;

                case PS:
                    Robot.getInstance().fwFlap.setPosition(.52);
                    break;

                case NULL:
                    break;
            }

            currentRampPos = wantedRampPos;
        }

        if (currentMode != wantedMode) {
            switch (wantedMode) {
                case OFF:
                    Robot.getInstance().flyWheel.motor.setVelocity(0);
                    break;
                case DEFAULT:
                    setRPM(1500);
                    break;
                case POWERSHOT1:
                    setRPM(1250);
                    break;
                case POWERSHOT2:
                    setRPM(1250);
                    break;
                case POWERSHOT3:
                    setRPM(1250);
                    break;
                case POWERSHOTAUTO1:
                    setRPM(1250 + 100);
                case POWERSHOTAUTO2:
                    setRPM(1250 + 65);
                case POWERSHOTAUTO3:
                    setRPM(1250 + 80);
            }

            currentMode = wantedMode;
        }

        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);
    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    public boolean shootRing() {
        if (shootTimer.milliseconds() > time) {
            shootTimer.reset();
            if (extend) {
                Robot.getInstance().fwArm.setPosition(0.47);
                extend = false;
                return true;
            } else {
                Robot.getInstance().fwArm.setPosition(.62);
                extend = true;
            }
        }
        return false;
    }

    public int counter = 0;
    public double power;
    public void setRPM(double rpm) {
        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
    }

    public double getRPM() {
        return Robot.getInstance().flyWheel.getVelocity() / 28 * 60;
    }

    public void extend() {
        Robot.getInstance().fwArm.setPosition(0.62);
    }

    public void retract() {
        Robot.getInstance().fwArm.setPosition(0.47);
    }

    public void locked() {
        Robot.getInstance().fwLock.setPosition(1);
    }

    public void unlocked() {
        Robot.getInstance().fwLock.setPosition(.4);
    };

    public void setRampUp () {
        wantedRampPos = RampPositions.UP;
    }

    public void setRampDown () {
        wantedRampPos = RampPositions.DOWN;
    }

    public void setRampPS () {
        wantedRampPos = RampPositions.PS;
    }
}
