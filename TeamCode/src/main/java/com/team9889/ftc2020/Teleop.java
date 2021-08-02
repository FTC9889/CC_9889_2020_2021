package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.teleop.FindGoal;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Intake;
import com.team9889.ftc2020.subsystems.WobbleGoal;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    public static int fwTolerance = 80, fwHighTolerance = 90;

    int ready = 0, psWait = 10;
    boolean wgFirst = true, psFirst = true, wgTimerReset;

    boolean first = true, second = true, third = true, fourth = true, ringShot = false;

    Action psAction;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);

        waitForStart(false);

        Robot.getCamera().wantedCamState = Camera.CameraStates.GOAL;

        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;

        Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;

        Robot.getWobbleGoal().wantedArmPos = WobbleGoal.wgArmPositions.IN;

        Robot.camera.setPipeline(Robot.getCamera().scanForGoal);
        ThreadAction(new FindGoal());

        while(opModeIsActive()) {
            if (gamepad2.left_stick_button) {
                Robot.blue = true;
                Robot.getCamera().scanForGoal.blueGoal();
            } else if (gamepad2.right_stick_button) {
                Robot.blue = false;
                Robot.getCamera().scanForGoal.redGoal();
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
//                --------------------
//                |      Drive       |
//                --------------------
                if (!driverStation.getAim()) {
                    Robot.getMecanumDrive().xSpeed += driverStation.getX();
                    Robot.getMecanumDrive().ySpeed += driverStation.getY();
                    Robot.getMecanumDrive().turnSpeed += driverStation.getSteer();
                }

                if (gamepad1.dpad_down) {
                    Robot.getMecanumDrive().resetPos = true;
                } else {
                    Robot.getMecanumDrive().resetPos = false;
                }

                if (gamepad2.a) {
                    Robot.getMecanumDrive().move();
                } else
                    Robot.getMecanumDrive().setInitPowerShotPos();

//                --------------------
//                |      Intake      |
//                --------------------

                //TODO Turn intake motors to run on enum
                if (driverStation.getStartFrontIntaking()) {
                    Robot.getIntake().frontIntakeOn = true;
                    Robot.getIntake().backIntakeOn = false;
                    Robot.getIntake().passThroughIntakeOn = true;
                    Robot.getIntake().outtake = false;
                } else if (driverStation.getStartBackIntaking()) {
                    Robot.getIntake().frontIntakeOn = false;
                    Robot.getIntake().backIntakeOn = true;
                    Robot.getIntake().passThroughIntakeOn = true;
                    Robot.getIntake().outtake = false;
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().frontIntakeOn = true;
                    Robot.getIntake().backIntakeOn = true;
                    Robot.getIntake().passThroughIntakeOn = true;
                    Robot.getIntake().outtake = true;
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().frontIntakeOn = false;
                    Robot.getIntake().backIntakeOn = false;
                    Robot.getIntake().passThroughIntakeOn = false;
                    Robot.getIntake().outtake = false;
                }


                if (driverStation.getArmsDown()) {
                    Robot.getIntake().wantedArmPos = Intake.ArmPositions.DOWN;
                } else if (driverStation.getArmsUp()) {
                    Robot.getIntake().wantedArmPos = Intake.ArmPositions.UP;
                }


//                --------------------
//                |     FlyWheel     |
//                --------------------
                if (!Robot.getFlyWheel().autoPower) {
                    if (driverStation.getFW()) {
                        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.DEFAULT;
                        driverStation.psOn = false;
//                        Robot.getFlyWheel().time = 100;
                    } else if (!Robot.getFlyWheel().autoPower && !driverStation.getPS()) {
                        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.OFF;
                        driverStation.fwOn = false;
                        driverStation.psOn = false;
//                        Robot.getFlyWheel().time = 100;
                    }

                    if (driverStation.getPS()) {
                        Robot.getFlyWheel().wantedMode = FlyWheel.Mode.POWERSHOT1;
                        driverStation.fwOn = false;
                        Robot.getFlyWheel().time = 300;
                    }

//                    gamepad1.right_bumper

                    if (driverStation.getAim()) {
                        driverStation.fwOn = true;
                        driverStation.psOn = false;
                    }

                    if (driverStation.getAim() || driverStation.getShoot()) {
                        if (ready > 5 || Robot.getFlyWheel().shooting || driverStation.getShoot()) {
                            if (!ringShot) {
                                if (driverStation.psOn) {
//                                    ringShot = Robot.getFlyWheel().shootRing(50, fwHighTolerance);
                                    ringShot = Robot.getFlyWheel().shootRing(50);
                                } else {
//                                    ringShot = Robot.getFlyWheel().shootRing(fwTolerance, fwHighTolerance);
                                    ringShot = Robot.getFlyWheel().shootRing(fwTolerance, fwHighTolerance);
                                }
                            }

                            Robot.getFlyWheel().shooting = true;

                            Robot.getIntake().passThroughIntakeOnPartial = true;

                            if (ringShot) {
                                if (driverStation.getAutoPS() && psWait == 0) {
                                    if (first) {
                                        first = false;
                                        ready = 0;
                                        psWait = 10;
                                        ringShot = false;
                                        Robot.getFlyWheel().shooting = false;
                                    } else if (third) {
                                        third = false;
                                        ready = 0;
                                        psWait = 10;
                                        ringShot = false;
                                        Robot.getFlyWheel().shooting = false;
                                    } else if (second) {
                                        second = false;
                                        ready = 0;
                                        psWait = 10;
                                        ringShot = false;
                                        Robot.getFlyWheel().shooting = false;
                                    }
                                } else if (!driverStation.getAutoPS()) {
                                    ringShot = false;
                                    Robot.getFlyWheel().shooting = false;

//                                    if (Robot.hopperDist.getDistance(DistanceUnit.INCH) > 2.5) {
//                                        driverStation.codeToggle = !driverStation.codeToggle;
//                                    }
                                }

                                if (driverStation.getAutoPS())
                                    psWait--;
                            }
                            Robot.getFlyWheel().unlocked();

                            if (gamepad2.right_trigger < .5)
                                Robot.rr.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        } else {
                            if (driverStation.getAutoPS()) {
                                Robot.getFlyWheel().unlocked();
                                if (first) {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, -22), new Vector2d(0, 0));
                                } else if (third) {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, -8), new Vector2d(0, 0));
                                } else if (fourth) {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, 10), new Vector2d(0, 0));
                                    fourth = false;
                                } else if (second) {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, -18), new Vector2d(0, 0));
                                }
                            } else {
                                if (Robot.blue) {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, 36), new Vector2d(0, 0));
                                } else {
                                    Robot.getMecanumDrive().turn(new Vector2d(73, -36), new Vector2d(0, 0));
//                                    Robot.getMecanumDrive().turn(new Vector2d(73, -42), true);
                                }
                            }
                        }

                    } else {
//                        Robot.getMecanumDrive().theta = 1000;
                        psWait = 10;
                        Robot.getFlyWheel().locked();
                        Robot.getFlyWheel().shooting = false;
                        ringShot = false;

                        Robot.getIntake().passThroughIntakeOnPartial = false;

//                        driverStation.codeToggle = false;

                        Robot.fwArm.setPosition(0.47);

//                        first = true;
//                        second = true;
//                        third = true;
                    }

                    if (driverStation.getPS()) {
                        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.PS;
                    } else if (!Robot.getFlyWheel().autoPower) {
                        Robot.getFlyWheel().wantedRampPos = FlyWheel.RampPositions.DOWN;
                    }
                }

                if (Robot.blue == driverStation.getMiddleGoal()) {
                    Robot.getCamera().scanForGoal.redGoal();
                } else {
                    Robot.getCamera().scanForGoal.blueGoal();
                }

//                --------------------
//                |    WobbleGoal    |
//                --------------------
                if (gamepad1.left_bumper && wgTimerReset) {
                    Robot.getWobbleGoal().wgTimer.reset();
                    wgFirst = false;
                    wgTimerReset = false;
                } else if (!gamepad1.left_bumper) {
                    wgTimerReset = true;
                }

                if (driverStation.getWG() && !wgFirst) {
                    Robot.getWobbleGoal().pickUpWG();
                } else if (!wgFirst) {
                    Robot.getWobbleGoal().putWGDown();
                }
//                if (Robot.getWobbleGoal().currentArmPos != WobbleGoal.wgArmPositions.DOWN)

                if (driverStation.getDropWG()) {
                    Robot.getWobbleGoal().wantedGrabberOpen = true;
                }

//                --------------------
//                |      Camera      |
//                --------------------
                double angle = -Robot.getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
                if (angle < 0) {
                    angle += 360;
                }
                if (Math.abs(Math.toDegrees(Robot.getMecanumDrive().theta) - angle) < 4) {
                    ready++;
                } else {
                    ready = 0;
                }

            } else {
                Robot.getMecanumDrive().writeAngleToFile();
            }

            Robot.getCamera().getRobotPos();
            telemetry.addData("High Tolerance", fwHighTolerance + Robot.getFlyWheel().rpm);
            telemetry.addData("Low Tolerance", Robot.getFlyWheel().rpm - fwTolerance);

            telemetry.addData("Speed", Robot.getFlyWheel().rpm);

            telemetry.addData("Blue", Robot.blue);

            telemetry.addData("Auto Aim", driverStation.getAim());
            telemetry.addData("Hopper Distance", Robot.hopperDist.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Code Toggle", driverStation.codeToggle);

            telemetry.addData("First", first + ", Second : " + second + ", Third : " + third);

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
