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

    private enum RingStop {
        Closed, Open
    }

    private enum FingerState {
        Extended, Retracted
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
        // pid.update(Robot.getInstance().flyWheel.getVelocity(), rpm);
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
    // Kinda only allows for continual shooting or the lock will close
    // Checks if the rpm is within a tolerance before shooting
    public void fireControl(boolean shoot) {
        if(shoot) {
            // Check if the lock has been released and if it hasn't, open it
            if (currentLockState == RingStop.Open && isLockFinishedMoving()) {
                // Check if the finger has loaded a ring to push, and if the flywheel is up to speed
                if (currentFingerState == FingerState.Retracted && isFingerFinishedMoving()
                        && isRPMInTolerance(10))
                    setPusherState(FingerState.Extended);
                // Check if the finger has pushed a ring into the flywheel and if if has moved all the way
                else if (currentFingerState == FingerState.Extended && isFingerFinishedMoving())
                    setPusherState(FingerState.Retracted);
            } else {
                setLockState(RingStop.Open);
            }
        } else {
            // Check if the has cleared the flywheel and if it has, lock the rings in
            if(currentFingerState == FingerState.Extended && isFingerFinishedMoving()) {
                setPusherState(FingerState.Retracted);
            } else if(currentFingerState == FingerState.Retracted && isFingerFinishedMoving()) {
                setLockState(RingStop.Closed);
            }
        }
    }


    // TODO: Fix these values so we don't make direct calls to hardware in teleop thread
    private void extend() {
        Robot.getInstance().fwArm.setPosition(0.45);
    }

    // TODO: Fix these values so we don't make direct calls to hardware in teleop thread
    private void retract() {
        Robot.getInstance().fwArm.setPosition(1.0);
    }

    // Set Pusher State
    private ElapsedTime pusherTimer = new ElapsedTime();
    private FingerState currentFingerState = FingerState.Extended;
    private void setPusherState(FingerState state) {
        switch (state) {
            case Retracted:
                retract();
                break;
            case Extended:
                extend();
                break;
        }

        if(currentFingerState != state)
            pusherTimer.reset();

        currentFingerState = state;
    }

    // Time for finger to move to position
    private boolean isFingerFinishedMoving() {
        if (currentFingerState == FingerState.Extended)
            return pusherTimer.milliseconds() > 100;
        else
            return pusherTimer.milliseconds() > 50;
    }

    // Set Lock State
    private ElapsedTime lockServoTimer = new ElapsedTime();
    private RingStop currentLockState = RingStop.Closed;
    public void setLockState(RingStop state) {
        switch (state) {
            case Open:
                Robot.getInstance().fwLock.setPosition(.4);
                break;
            case Closed:
                Robot.getInstance().fwLock.setPosition(1);
                break;
        }

        if(currentLockState != state)
            lockServoTimer.reset();

        currentLockState = state;
    }

    private boolean isLockFinishedMoving() {
        return lockServoTimer.milliseconds() > 500;
    }
}
