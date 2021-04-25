package com.team9889.ftc2020.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.CruiseLib;
import com.team9889.lib.android.FileReader;
import com.team9889.lib.android.FileWriter;
import com.team9889.lib.control.controllers.PID;
import com.team9889.lib.control.controllers.PIDF;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * Created by joshua9889 on 3/28/2018.
 */

@Config
public class FlyWheel extends Subsystem{
    public enum Mode {
        OFF, POWERSHOT1, POWERSHOT2, POWERSHOT3, POWERSHOTAUTO1, POWERSHOTAUTO2, POWERSHOTAUTO3, DEFAULT
    }

    public enum RingStop {
        Closed, Open
    }

    public static double P = 115, I = 0, D = 1.5, F = 0;
    public static int ps1 = 1215, ps2 = 1160, ps3 = 1120;
    public PIDF pid = new PIDF(81, 0, 1, 0);

    public boolean done = false;

    public boolean psPower = false;

    private double targetVelocity = 0;

    @Override
    public void init(boolean auto) {
        updatePIDF(P, I, D, F);

        if (auto) {
            setLockState(RingStop.Closed);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {}

    @Override
    public void update() {
        pid.p = P;
        pid.i = I;
        pid.d = D;
        pid.kFF = F;

        updatePIDF(P, I, D, F);
    }

    @Override
    public void stop() {
        setMode(Mode.OFF);
    }

    public void setMode(Mode mode) {
        switch (mode) {
            case OFF:
                setRPM(0);
                break;
            case DEFAULT:
                setRPM(2540);
                break;
            case POWERSHOT1:
                setRPM(ps1);
                break;
            case POWERSHOT2:
                setRPM(ps2);
                break;
            case POWERSHOT3:
                setRPM(ps3);
                break;
            case POWERSHOTAUTO1:
                setRPM(ps1 + 100);
            case POWERSHOTAUTO2:
                setRPM(ps2 + 65);
            case POWERSHOTAUTO3:
                setRPM(ps3 + 80);
        }
    }

    public void setRPM(double rpm) {
//        pid.update(Robot.getInstance().flyWheel.getVelocity(), rpm);
        targetVelocity = rpm;
        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
    }

    // Set the pidf constants easier
    public void updatePIDF(double p, double i, double d, double f) {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    // Calculate the percent error
    // determine if it is within a percent tolerance of the wanted rpm
    public boolean isRPMInTolerance(double percent) {
        return percent > 50 * (Robot.getInstance().flyWheel.getVelocity() - targetVelocity);
    }

    // IDK if this will work or not
    private ElapsedTime fireTimer = new ElapsedTime();
    public void fire() {
        if(fireTimer.milliseconds() > 120) {
            retract();
            fireTimer.reset();
        } else {
            extend();
        }
    }

    // TODO: Fix these values so we don't make direct calls to hardware in teleop thread
    public void extend() {
        Robot.getInstance().fwArm.setPosition(0.45);
    }

    public void retract() {
        Robot.getInstance().fwArm.setPosition(1.0);
    }

    public void setLockState(RingStop state) {
        switch (state) {
            case Open:
                Robot.getInstance().fwLock.setPosition(.4);
                break;
            case Closed:
                Robot.getInstance().fwLock.setPosition(1);
                break;
        }
    }
}
