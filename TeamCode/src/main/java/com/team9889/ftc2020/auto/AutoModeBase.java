package com.team9889.ftc2020.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.Team9889Linear;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.utl.RobotUpdate;

/**
 * Created by joshua9889 on 8/5/2017.
 */

public abstract class AutoModeBase extends Team9889Linear {

    // Autonomous Settings
    private StartPosition currentAutoRunning = StartPosition.REDRIGHT;

    // Timer for autonomous
    protected ElapsedTime autoTimer = new ElapsedTime();
    double lastLoop = 0;

    public enum StartPosition {
        REDRIGHT, REDLEFT, BLUERIGHT, BLUELEFT, PARK;

        private static String redString = "Red";
        private static String blueString = "Blue";

        private static int RED_Num = 1;
        private static int BLUE_Num = -1;
    }

    public enum Boxes {
        CLOSE, MIDDLE, FAR
    }
    public Boxes box;

    // Checks for a saved file to see what auto we are running?
    // TODO: Use gamepad or maybe camera to select which auto to run
    private void setCurrentAutoRunning(){
        currentAutoRunning = side();
    }

    // Method to implement in the auto to run the autonomous
    public abstract void run(StartPosition startPosition, Boxes box);

    public abstract StartPosition side();

    @Override
    public void runOpMode() throws InterruptedException{
        setCurrentAutoRunning();

        waitForStart(true, currentAutoRunning);
        autoTimer.reset();

        box = Robot.getCamera().getRSBox();

        ThreadAction(new RobotUpdate());

        // If the opmode is still running, run auto
        if (opModeIsActive() && !isStopRequested()) {
            run(currentAutoRunning, box);
        }

        // Stop all movement
        finalAction();
    }


    /**
     * Run a single action, once, thread-blocking
     * @param action Action Class wanting to run
     */
    public void runAction(Action action){
        if(opModeIsActive() && !isStopRequested()) {
            action.start();

            while (!action.isFinished() && opModeIsActive() && !isStopRequested()) {
                action.update();
//            Robot.getMecanumDrive().setFieldCentricAutoPower(Robot.getMecanumDrive().xSpeed,
//                    Robot.getMecanumDrive().ySpeed, Robot.getMecanumDrive().turnSpeed);
//            Robot.update();
//                Robot.outputToTelemetry(telemetry);
//                telemetry.update();
//                while (autoTimer.milliseconds() - lastLoop < 10) {
//
//                }
//                lastLoop = autoTimer.milliseconds();
            }

            if (opModeIsActive() && !isStopRequested()) {
                action.done();
            }
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