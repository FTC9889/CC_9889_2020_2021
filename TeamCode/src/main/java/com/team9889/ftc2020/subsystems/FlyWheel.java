package com.team9889.ftc2020.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.lib.CircularBuffer;
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

    double time = 200;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean extend = false;

    public static double P = 150, I = 0, D = 20, F = 0.3;
    public static int ps1 = 1250, ps2 = 1250, ps3 = 1250;
    public PIDF pid = new PIDF(81, 0, 1, 0);

    public static double voltageConstant = 1;

    public boolean done = false;

    public boolean psPower = false;
    public CircularBuffer KfEstimator = new CircularBuffer(6);

    @Override
    public void init(boolean auto) {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

        if (auto) {
            Robot.getInstance().fwLock.setPosition(1);
            Robot.getInstance().fwFlap.setPosition(0.7);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {}

    @Override
    public void update() {
        Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);
    }

    @Override
    public void stop() {
        Robot.getInstance().flyWheel.setPower(0);
    }

    public void setMode(Mode mode) {
        switch (mode) {
            case OFF:
                Robot.getInstance().flyWheel.motor.setVelocity(0);
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
        /*
        if (counter < 10) {
            pid.p = P;
            pid.i = I;
            pid.d = D;
            pid.kFF = F;
            Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(P, I, D, F);

            power += pid.update(getRPM(), rpm);
            Log.i("PID", "" + pid.getOutput());
        } else if (counter >= 10 && counter < 20) {
            KfEstimator.addValue(estimateKf(getRPM(), Robot.getInstance().result));
        }
        if (counter >= 20) {
            if (getRPM() > rpm) {
                KfEstimator.addValue(estimateKf(getRPM(), Robot.getInstance().result));
            }
            pid.p = 0;
            pid.i = 0;
            pid.d = 0;
            pid.kFF = KfEstimator.getAverage();
            Robot.getInstance().flyWheel.motor.setVelocityPIDFCoefficients(0, 0, 0, KfEstimator.getAverage());

            power += pid.update(getRPM(), rpm);
            Log.i("PID", "" + pid.getOutput());
        }
        Log.i("Counter", "" + counter);

        if (rpm - getRPM() < 150) {
            counter++;
        } else {
            counter = 0;
        }
        */

        Robot.getInstance().flyWheel.motor.setVelocity(rpm);
//        Robot.getInstance().flyWheel.setPower(CruiseLib.limitValue(pid.getOutput(), 1, 0));
    }

    /**
     * Estimate the kF value from current RPM and voltage
     */
    private double estimateKf(double rpm, double voltage) {
        final double speed_in_ticks_per_100ms = 28 * (rpm / 600);
        final double output = voltageConstant / 12.0 * voltage;
        return output / speed_in_ticks_per_100ms;
    }

    public double getRPM() {
        return Robot.getInstance().flyWheel.getVelocity() / 28 * 60;
    }

    public void fire() {

    }

    public void extend() {
        Robot.getInstance().fwArm.setPosition(0.45);
    }

    public void retract() {
        Robot.getInstance().fwArm.setPosition(1.0);
    }
}
