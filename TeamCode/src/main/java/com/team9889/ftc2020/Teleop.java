package com.team9889.ftc2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Intake;
import com.team9889.ftc2020.subsystems.WobbleGoal;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;

        Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;

        while(opModeIsActive()) {
            if (gamepad2.left_stick_button) {
                Robot.blue = true;
            } else if (gamepad2.right_stick_button) {
                Robot.blue = false;
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
//                --------------------
//                |      Drive       |
//                --------------------
                Robot.getMecanumDrive().xSpeed += driverStation.getX();
                Robot.getMecanumDrive().ySpeed += driverStation.getY();
                Robot.getMecanumDrive().turnSpeed += driverStation.getSteer();


//                --------------------
//                |      Intake      |
//                --------------------
                if (driverStation.getStartIntaking()) {
                    Robot.getIntake().frontIntakeOn = true;
                    Robot.getIntake().backIntakeOn = false;
                    Robot.getIntake().passThroughIntakeOn = true;
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().frontIntakeOn = false;
                    Robot.getIntake().backIntakeOn = true;
                    Robot.getIntake().passThroughIntakeOn = true;
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().frontIntakeOn = false;
                    Robot.getIntake().backIntakeOn = false;
                    Robot.getIntake().passThroughIntakeOn = false;
                }


                if (driverStation.getArmsDown()) {
                    Robot.getIntake().wantedArmPos = Intake.ArmPositions.DOWN;
                } else if (driverStation.getArmsUp()) {
                    Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;
                }


//                --------------------
//                |     FlyWheel     |
//                --------------------
                if (driverStation.getFW()) {
                    Robot.getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;
                } else if (driverStation.getPS()) {
                    Robot.getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT1;
                } else if (!Robot.getFlyWheel().autoPower){
                    Robot.getFlyWheel().wantedMode = FlyWheel.Mode.OFF;
                }

                //Add shooting

                if (driverStation.getPS()) {
                    Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;
                } else if (!Robot.getFlyWheel().autoPower) {
                    Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;
                }

//                --------------------
//                |    WobbleGoal    |
//                --------------------
                if (driverStation.getWG() && Robot.getWobbleGoal().currentArmPos != WobbleGoal.wgArmPositions.UP) {
                    Robot.getWobbleGoal().pickUpWG();
                } else if (Robot.getWobbleGoal().currentArmPos != WobbleGoal.wgArmPositions.DOWN){
                    Robot.getWobbleGoal().putWGDown();
                }

                if (driverStation.getDropWG()) {
                    Robot.getWobbleGoal().wantedGrabberOpen = true;
                }

//                --------------------
//                |      Camera      |
//                --------------------
            }

        }
    }
}
