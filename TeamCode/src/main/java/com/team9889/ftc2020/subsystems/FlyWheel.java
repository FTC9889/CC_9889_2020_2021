package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.control.controllers.FFFBMath;
import com.team9889.lib.control.controllers.PIDF;
import com.team9889.lib.control.controllers.PIDMath;

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

    public static double time = 100, rpm = 3000;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean extend = false;

    public static double P = 0.002, I = 0, D = 0, F = 0, V = 0.0002, A, S;
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
//        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

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

//        if (currentMode != wantedMode) {
            switch (wantedMode) {
                case OFF:
//                    Robot.getInstance().flyWheel.motor.setVelocity(0);
                    setRPM(0);
                    break;
                case DEFAULT:
                    setRPM(rpm);
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
//        }

//        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        PMath.PIDConstants(P, I, D);
        FFMath.FFConstants(V, A, S);
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

    FFFBMath FFMath = new FFFBMath(0, 0, 0);
    PIDMath PMath = new PIDMath(0, 0, 0);

    public int counter = 0;
    public double power;
    public void setRPM(double rpm) {
//        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
        double power = FFMath.calculateFFFBGain(rpm)+PMath.calculateGain(rpm - (Robot.getInstance().flyWheel.getVelocity() / 28 * 60));
        power = CruiseLib.limitValue(power, 1, 0);
        Log.i("Power", "" + power);

        Robot.getInstance().flyWheel.setPower(power);
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
        Robot.getInstance().fwLock.setPosition(.5);
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
