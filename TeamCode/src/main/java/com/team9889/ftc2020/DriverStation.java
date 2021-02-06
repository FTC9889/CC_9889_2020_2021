package com.team9889.ftc2020;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    double getLiftPower(boolean isDown) {
        double g1UpPower = gamepad1.right_trigger;
        double g2UpPower = gamepad2.left_trigger;
        double g1DownPower = gamepad1.left_trigger;
        double g2DownPower = gamepad2.right_trigger;

        if(g1UpPower > 0.1)
            return -g1UpPower;
        else if(g1DownPower > 0.1 && !isDown)
            return g1DownPower;
        else if(g2UpPower > 0.1)
            return -g2UpPower;
        else if(g2DownPower > 0.1 && !isDown)
            return g2DownPower;
        else
            return 0;
    }

    boolean getReleaseStone(){
        return gamepad2.right_bumper || gamepad1.right_bumper;
    }

    boolean getStartIntaking(){
        return gamepad1.a;
    }

    boolean getStopIntaking() {
        return gamepad1.b;
    }

    boolean getStartOuttaking() {
        return gamepad1.y;
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

    private boolean wggToggle = true;
    private boolean wggOpen = true;
    boolean getWGG() {
        if(gamepad1.dpad_up && wggToggle) {
            wggOpen = !wggOpen;
            wggToggle = false;
        } else if(!gamepad1.dpad_up)
            wggToggle = true;

        return wggOpen;
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


    private boolean slowDownToggle = true;
    private boolean slowDown = false;
    double getSlowDownFactor() {
        if(gamepad1.back && slowDownToggle) {
            slowDown = !slowDown;
            slowDownToggle = false;
        } else if(!gamepad1.back)
            slowDownToggle = true;

        return slowDown ? 3 : 1;
    }

    private boolean fwToggle = true;
    private boolean fwOn = false;
    boolean getFW() {
        if(gamepad1.x && fwToggle) {
            fwOn = !fwOn;
            fwToggle = false;
        } else if(!gamepad1.x)
            fwToggle = true;

        return fwOn;
    }

//    private boolean foundationToggle = false;
//    private boolean foundationClose = false;
//    boolean getFoundationClose() {
//        if(gamepad2.dpad_up && foundationToggle) {
//            foundationClose = !foundationClose;
//            foundationToggle = false;
//        } else if(!gamepad2.dpad_up)
//            foundationToggle = true;
//
//        return foundationClose;
//    }

    private boolean grabberToggle = true;
    private boolean grabberOpen = true;
    boolean getGrabberOpen(boolean override) {
        if(gamepad2.y && grabberToggle) {
            grabberOpen = !grabberOpen;
            grabberToggle = false;
        } else if(!gamepad2.y)
            grabberToggle = true;

        if(gamepad1.dpad_right)
            grabberOpen = false;

        if(!override || gamepad1.a || gamepad1.y)
            grabberOpen = true;

        return grabberOpen;
    }

    private boolean linearBarToggle = false;
    private boolean linearBarIn = true;
    boolean getLinearBarIn(boolean override) {
        if((gamepad2.right_bumper || gamepad1.left_bumper) && linearBarToggle) {
            linearBarIn = !linearBarIn;
            linearBarToggle = false;
        } else if(!gamepad2.right_bumper && !gamepad1.left_bumper)
            linearBarToggle = true;

        if(!override)
            linearBarIn = true;

        return linearBarIn;
    }

    boolean scoreStone() {
        return gamepad1.dpad_left;
    }

    private boolean capStoneToggle = true;
    private boolean capStoneDeployed = false;
    boolean capStone(boolean override) {
        if ((gamepad2.left_bumper) && capStoneToggle) {
            capStoneDeployed = !capStoneDeployed;
            capStoneToggle = false;
        } else if (!gamepad2.left_bumper)
            capStoneToggle = true;

        if(!override)
            capStoneToggle = true;

        return capStoneDeployed;
    }

    private boolean capStoneAutoToggle = true;
    private boolean capStoneAutoDeployed = false;
    boolean capStoneAuto(boolean override, boolean done) {
        if ((gamepad2.a) && capStoneAutoToggle) {
            capStoneAutoDeployed = !capStoneAutoDeployed;
            capStoneAutoToggle = false;
        } else if (done){
            capStoneAutoDeployed = false;
            linearBarIn = true;
            grabberOpen = true;
            capStoneDeployed = true;
        } else if (!gamepad2.a)
            capStoneAutoToggle = true;

        if(!override)
            capStoneAutoToggle = true;

        return capStoneAutoDeployed;
    }

    boolean releaseTapeMeasure() {
        return gamepad2.dpad_down;
    }

    boolean resetIMU() {
        return gamepad1.right_stick_button && gamepad1.left_stick_button;
    }
}
