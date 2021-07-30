package com.team9889.ftc2020;

import com.qualcomm.robotcore.hardware.Gamepad;

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

    boolean getStartFrontIntaking(){
        return gamepad1.a;
    }

    boolean getStartBackIntaking(){
        return gamepad1.x;
    }

    boolean getStopIntaking() {
        return gamepad1.b || gamepad2.left_trigger > .3;
    }

    boolean getStartOuttaking() {
        return gamepad1.y;
    }

    boolean getArmsDown() {return gamepad2.dpad_down;}

    boolean getArmsUp()  {return gamepad2.dpad_left;}

    boolean getDropWG() {return gamepad2.right_bumper;}

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
    public boolean fwOn = false;
    boolean getFW() {
        if(gamepad1.right_trigger > 0.3 && fwToggle) {
            fwOn = !fwOn;
            fwToggle = false;
        } else if(gamepad1.right_trigger <= 0.1)
            fwToggle = true;

        return fwOn;
    }

//    private boolean aimToggle = true;
//    private boolean aimOn = false;
//    public boolean codeToggle = false;
//    boolean getAim() {
//        if((gamepad2.left_bumper || codeToggle) && aimToggle) {
//            aimOn = !aimOn;
//            aimToggle = false;
//            codeToggle = false;
//        } else if(!gamepad2.left_bumper && !codeToggle)
//            aimToggle = true;
//
//        return aimOn;
//    }

    boolean getAim() {
        return gamepad2.left_bumper;
    }

    boolean getShoot() {
        return gamepad1.right_bumper;
    }

    private boolean psToggle = true;
    public boolean psOn = false;
    boolean getPS() {
        if(gamepad1.left_trigger > 0.3 && psToggle) {
            psOn = !psOn;
            psToggle = false;
        } else if(gamepad1.left_trigger <= 0.1)
            psToggle = true;

        return psOn;
    }

    boolean getAutoPS () {return gamepad1.dpad_right;}

    private boolean psFirstToggle = true;
    private boolean psFirst = true;
    boolean getPSFirst() {
        if(gamepad2.a && psFirstToggle) {
            psFirst = !psFirst;
            psFirstToggle = false;
        } else if(!gamepad2.a)
            psFirstToggle = true;

        return psFirst;
    }

    private boolean psSecondToggle = true;
    private boolean psSecond = true;
    boolean getPSSecond() {
        if(gamepad2.b && psSecondToggle) {
            psSecond = !psSecond;
            psSecondToggle = false;
        } else if(!gamepad2.b)
            psSecondToggle = true;

        return psSecond;
    }

    private boolean psThirdToggle = true;
    private boolean psThird = false;
    boolean getMiddleGoal() {
        if(gamepad2.y && psThirdToggle) {
            psThird = !psThird;
            psThirdToggle = false;
        } else if(!gamepad2.y)
            psThirdToggle = true;

        return psThird;
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

    private boolean goalToggle = true;
    private boolean middleGoal = false;
    boolean getGoal() {
        if(gamepad2.x && goalToggle) {
            middleGoal = !middleGoal;
            goalToggle = false;
        } else if(!gamepad2.x)
            goalToggle = true;

        return middleGoal;
    }

    boolean resetIMU() {
        return gamepad1.right_stick_button && gamepad1.left_stick_button;
    }


}
