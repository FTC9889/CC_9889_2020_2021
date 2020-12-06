package com.team9889.ftc2019.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2019.Team9889Linear;
import com.team9889.ftc2019.auto.actions.Action;
import com.team9889.ftc2019.auto.actions.utl.RobotUpdate;

/**
 * Created by joshua9889 on 8/5/2017.
 */

public abstract class AutoModeBase extends Team9889Linear {

    // Autonomous Settings
    private Side currentAutoRunning = AutoModeBase.Side.RED;

    // Timer for autonomous
    protected ElapsedTime autoTimer = new ElapsedTime();

    protected enum Side {
        RED, BLUE;

        private static String redString = "Red";
        private static String blueString = "Blue";

        private static int RED_Num = 1;
        private static int BLUE_Num = -1;

        public static int getNum(Side side){
            return side == RED ? RED_Num : BLUE_Num;
        }

        public static Side fromText(String side) {
            return side.equals(redString) ? RED : BLUE;
        }
    }

    // Test getting the Side number
    public static void main(String... args) {
        com.team9889.ftc2019.auto.AutoModeBase.Side side = AutoModeBase.Side.BLUE;

        System.out.println(AutoModeBase.Side.getNum(side));
    }

    // Checks for a saved file to see what auto we are running?
    // TODO: Use gamepad or maybe camera to select which auto to run
    private void setCurrentAutoRunning(){

    }

    // Method to implement in the auto to run the autonomous
    public abstract void run(Side side);

    @Override
    public void runOpMode() throws InterruptedException{
        setCurrentAutoRunning();

        waitForStart(true);
        autoTimer.reset();

        ThreadAction(new RobotUpdate());

        // If the opmode is still running, run auto
        if (opModeIsActive() && !isStopRequested()) {
            run(currentAutoRunning);
        }

        // Stop all movement
        finalAction();
    }


    /**
     * Run a single action, once, thread-blocking
     * @param action Action Class wanting to run
     */
    public void runAction(Action action){
        if(opModeIsActive() && !isStopRequested())
            action.start();

        while (!action.isFinished() && opModeIsActive() && !isStopRequested()) {
            action.update();
            Robot.outputToTelemetry(telemetry);
            telemetry.update();
        }

        if(opModeIsActive() && !isStopRequested()) {
            action.done();
        }
    }

    /**
     * Run a single action, once, in a new thread
     * Caution to make sure that you don't run more one action on the same subsystem
     * @param action Action Class wanting to run
     */
    public void ThreadAction(final Action action){
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                runAction(action);
            }
        };

        if(opModeIsActive() && !isStopRequested())
            new Thread(runnable).start();
    }
}