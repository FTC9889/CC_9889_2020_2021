package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2020.auto.actions.teleop.FindGoal;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Intake;
import com.team9889.ftc2020.subsystems.WobbleGoal;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    boolean shooting = false;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);

        waitForStart(false);

        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;

        Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;

        Robot.camera.setPipeline(Robot.getCamera().scanForGoal);
        ThreadAction(new FindGoal());

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

                if (gamepad1.dpad_down) {
                    Robot.getMecanumDrive().turn(new Vector2d(73, -30), new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y));
                }

//                --------------------
//                |      Intake      |
//                --------------------

                //TODO Turn intake motors to run on enum
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

                if (gamepad1.right_bumper) {
                    if (Math.abs(Math.abs(Math.toDegrees(Robot.getMecanumDrive().theta) - 360) - Robot.getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES)) < 3 || shooting) {
                        Robot.getFlyWheel().shootRing();
                        Robot.getFlyWheel().unlocked();

                        shooting = true;
                    } else {
                        Robot.getMecanumDrive().turn(new Vector2d(73, -36), new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y));
                    }

                } else {
                    Robot.getMecanumDrive().theta = 1000;
                    Robot.getFlyWheel().locked();
                    shooting = false;
                }

                //TODO Add shooting

                if (driverStation.getPS()) {
                    Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;
                } else if (!Robot.getFlyWheel().autoPower) {
                    Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;
                }

                if (gamepad2.left_bumper) {
                    Robot.getCamera().wantedCamState = Camera.CameraStates.GOAL;
                } else {
                    Robot.getCamera().wantedCamState = Camera.CameraStates.NULL;
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
            } else {
                Robot.getMecanumDrive().writeAngleToFile();
            }

            telemetry.addData("Blue", Robot.blue);

            Robot.outputToTelemetry(telemetry);

            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            Robot.update();

            Robot.rr.getLocalizer().update();
        }
    }
}
