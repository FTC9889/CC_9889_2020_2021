package com.team9889.ftc2020.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Eric on 8/19/2019.
 */

public class Intake extends Subsystem {

    public enum IntakeState {
        Stop, Front, Back, Both, Outtake, Idle
    }

    public enum ArmState {
        Retracted, Extended
    }

    private boolean allowIdleState = true;
    private boolean checkRingDetector = false;

    @Override
    public void init(boolean auto) {
        if (auto) {
            setArmState(ArmState.Retracted);
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    boolean ringFront = false, ringHalf = false;
    int ringsPassed = 0;
    ElapsedTime ringTimer = new ElapsedTime();

    @Override
    public void update() {
        if(ringDetected(800)) { // TODO: Tune this number
            ringsPassed++;
        }

//        if (ringDetected()) {
//            if (!ringFront) {
//                ringFront = true;
//            } else if (ringHalf) {
//                ringsPassed++;
//            }
//
//            ringTimer.reset();
//        } else if (ringFront) {
////                if (ringTimer.milliseconds() > 800) {
////                    ringFront = false;
////                    ringHalf = false;
////                    ringsPassed++;
////                }
//
//            ringHalf = true;
//        }
    }

    @Override
    public void stop() {
        setIntakeState(IntakeState.Stop);
    }

    public void setFrontIntakePower(double power){
        allowIdleState = power == 0;
        checkRingDetector = power > 0;

        Robot.getInstance().frontIntake.setPower(power);
    }

    public void setBackIntakePower(double power){
        Robot.getInstance().backIntake.setPower(power);
    }

    public void setPassThroughPower(double power) {
        Robot.getInstance().passThrough.setPower(power);
    }

    // Set the state of the intakes
    public void setIntakeState(IntakeState state) {
        switch (state) {
            case Stop:
                setFrontIntakePower(0);
                setBackIntakePower(0);
                setPassThroughPower(0);
                break;
            case Front:
                setFrontIntakePower(1);
                setBackIntakePower(0);
                setPassThroughPower(1);
                break;
            case Back:
                setFrontIntakePower(0);
                setBackIntakePower(1);
                setPassThroughPower(1);
                break;
            case Outtake:
                setFrontIntakePower(-1);
                setBackIntakePower(-1);
                setPassThroughPower(-1);
                break;
            case Idle:
                if(allowIdleState)
                    setPassThroughPower(0.4);
                break;
            case Both:
                setFrontIntakePower(1);
                setBackIntakePower(1);
                setPassThroughPower(1);
                break;
        }
    }

    public void setArmState(ArmState state) {
        switch (state) {
            case Extended:
                Robot.getInstance().leftArm.setPosition(0);
                Robot.getInstance().rightArm.setPosition(0);
                break;
            case Retracted:
                Robot.getInstance().leftArm.setPosition(1);
                Robot.getInstance().rightArm.setPosition(1);
                break;
        }
    }

    // Milliseconds, Only detect the front edge
    // Assume constant velocity in intake, therefor it will be a constant time to hopper
    // Checks to see if the front intake is in an On state, reduces the amount of calls to I2C sensor
    public boolean ringDetected(double timeForOneRingToPass) {
        if(ringTimer.milliseconds() > timeForOneRingToPass && checkRingDetector) {
            ringTimer.reset();
            return Robot.getInstance().ringDetector.getDistance(DistanceUnit.INCH) < 2.7;
        } else {
            return false;
        }
    }

    public void resetRingCount() {
        ringsPassed = 0;
    }

    public int ringsInHopper() {
        return ringsPassed;
    }

}