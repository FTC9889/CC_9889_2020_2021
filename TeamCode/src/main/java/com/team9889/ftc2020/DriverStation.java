package com.team9889.ftc2020;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.subsystems.Intake;

/**
 * Created by MannoMation on 12/14/2018.
 */
public class DriverStation {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public DriverStation(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    double getX(){
        return gamepad1.left_stick_x;
    }

    double getY() {
        return -gamepad1.left_stick_y;
    }

    double getSteer(){
        return gamepad1.right_stick_x;
    }

    private boolean wgToggle = true;
    private boolean wgDown = false;
    public  boolean codePress = false;
    boolean getWG() {
        if((gamepad1.left_bumper && wgToggle) || codePress) {
            wgDown = !wgDown;

            wgToggle = false;
            codePress = false;
        } else if(!gamepad1.left_bumper && !codePress)
            wgToggle = true;

        return wgDown;
    }

    private boolean intakeFlipToggle = true;
    private boolean intakeFlipDown = false;
    boolean getIntakeFlip() {
        if (gamepad1.right_bumper){
            intakeFlipDown = false;
        } else if(gamepad1.left_bumper && intakeFlipToggle) {
            intakeFlipDown = !intakeFlipDown;
            intakeFlipToggle = false;
        } else if(!gamepad1.left_bumper || gamepad1.right_bumper)
            intakeFlipToggle = true;
        return intakeFlipDown;
    }

    private boolean fwToggle = true;
    private boolean fwOn = false;
    boolean getFW() {
        if(gamepad1.right_trigger > 0.3 && fwToggle) {
            fwOn = !fwOn;
            fwToggle = false;
        } else if(gamepad1.right_trigger <= 0.1)
            fwToggle = true;

        return fwOn;
    }

    private boolean intakeToggle = true;
    private boolean intakeOn = false;
    boolean getIntake() {
        if(gamepad1.dpad_up && intakeToggle) {
            intakeOn = !intakeOn;
            intakeToggle = false;
        } else if(!gamepad1.dpad_up)
            intakeToggle = true;

        return intakeOn;
    }

    boolean resetIMU() {
        return gamepad1.right_stick_button && gamepad1.left_stick_button;
    }

    boolean shoot() {
        return gamepad1.right_bumper || gamepad2.left_bumper;
    }

    // Intake Control
    private boolean getStartIntaking(){
        return gamepad1.a;
    }

    private boolean getStopIntaking() {
        return gamepad1.b || gamepad2.left_trigger > .3;
    }

    private boolean getStartOuttaking() {
        return gamepad1.y;
    }

    private boolean backIntake() {
        return gamepad1.x;
    }

    private boolean intakeIdle() {
        return gamepad2.left_bumper;
    }

    private Intake.IntakeState currentWantedIntakeState = Intake.IntakeState.Stop;
    Intake.IntakeState getWantedIntakeState() {
        if (getStartIntaking()) {
            currentWantedIntakeState = Intake.IntakeState.Front;
        } else if (getStopIntaking()) {
            currentWantedIntakeState = Intake.IntakeState.Stop;
        } else if (getStartOuttaking()) {
            currentWantedIntakeState = Intake.IntakeState.Outtake;
        } else if (backIntake()) {
            currentWantedIntakeState = Intake.IntakeState.Back;
        } else if (intakeIdle()) {
            currentWantedIntakeState = Intake.IntakeState.Idle;
        }

        return currentWantedIntakeState;
    }

}
