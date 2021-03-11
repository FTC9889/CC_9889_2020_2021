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

    boolean getStartIntaking(){
        return gamepad1.a;
    }

    boolean getStopIntaking() {
        return gamepad1.b || gamepad2.left_trigger > .1;
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

    boolean resetIMU() {
        return gamepad1.right_stick_button && gamepad1.left_stick_button;
    }

    private boolean resetRobotPose = false;
    boolean resetRobotPose() {
        if (gamepad2.left_stick_button && gamepad2.right_stick_button && !resetRobotPose) {

            resetRobotPose = true;
        } else if(!gamepad2.left_stick_button && !gamepad2.right_stick_button)
            resetRobotPose = false;

        return resetRobotPose;
    }
}
