package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Wheels;
import org.firstinspires.ftc.teamcode.eocv.ExampleProcessor;
import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.vision.VisionPortal;

public class MainAutonomus {
    boolean isRed;

    Wheels wheels;
    Arm arm;
    Claw claw;
    Elevator elevator;
    Intake intake;

    VisionPortal portal;
    ExampleProcessor propProcessor;


    IMU imu;

    LinearOpMode opMode;
    int propPosition;

    public MainAutonomus(boolean isRed, LinearOpMode opMode) {
        this.isRed = isRed;
        this.opMode = opMode;
    }

    public void runPeuple() {

        //isRed = false;
        if (opMode.opModeInInit()) {
            imu = opMode.hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            wheels = new Wheels(opMode, imu);
            arm = new Arm(opMode);
            arm.initArm();
            claw = new Claw(opMode);
            claw.initClaw();
            elevator = new Elevator(opMode);
            elevator.initElevator();
            intake = new Intake(opMode);
            intake.initIntake();

            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            if (isRed) {
                propProcessor = new ExampleProcessor(opMode.telemetry, 1);
            } else {
                propProcessor = new ExampleProcessor(opMode.telemetry, 2);
            }
            portal = VisionPortal.easyCreateWithDefaults(
                    webcamName, propProcessor);
        }
        while (opMode.opModeInInit() && !opMode.isStopRequested()) {

            propPosition = propProcessor.getPropPlacement();
            opMode.telemetry.addData("prop", propPosition);
            opMode.telemetry.update();
            opMode.sleep(20);
        }
        wheels.driveForward(60, 0.8, 2);
        wheels.driveForward(15, 0.5, 2);

        if ((propPosition == 1 && !isRed) || (propPosition == 3 && isRed) && !opMode.isStopRequested()) {
            if (!isRed) {
                wheels.rotateByEncoder(-90, 0.5);
            }
            else {
                wheels.rotateByEncoder(90, 0.5);

            }
            wheels.driveForward(30, 0.5, 2);
            wheels.driveBackword(19, 0.5, 2);
        } else if ((propPosition == 3 && !isRed) || (propPosition == 1 && isRed) && !opMode.isStopRequested()) {
            if (!isRed) {
                wheels.rotateByEncoder(90, 0.5);
                wheels.driveRight(8, 0.5, 2);
            }
            else {
                wheels.rotateByEncoder(-90, 0.5);
                wheels.driveLeft(8, 0.5, 2);
            }
            wheels.driveForward(30, 0.5, 2);
            wheels.driveBackword(15, 0.5, 2);
        } else {
            wheels.driveForward(20, 0.5, 2);
            wheels.driveBackword(18, 0.5, 2);
        }

        claw.closeClawRight();
        arm.midArm();
        opMode.sleep(500);
        arm.closeArm();
        opMode.sleep(500);
        if ((!isRed && propPosition == 3) || (isRed && propPosition == 1)) {
            wheels.driveBackword(20, 0.5);
        } else {
            wheels.driveBackword(10, 0.5);
        }
    }

    public void blueYellowPixel() {
        if (propPosition == 2 && !opMode.isStopRequested()) {
            if (isRed) {
                wheels.rotateByEncoder(90, 0.5);
            } else {
                wheels.rotateByEncoder(-90, 0.5);
            }
        } else if (propPosition == 3 && !opMode.isStopRequested()) {
            if (isRed) {
                wheels.driveRight(31, 0.5);
            } else {
                wheels.rotateByEncoder(-180, 0.5);
            }
        } else {
            if (isRed) {
                wheels.rotateByEncoder(180, 0.5);
            } else {
                wheels.driveLeft(31, 0.5);
            }
        }
        arm.openArm();
//        elevator.elevatorUp(10);

        wheels.driveForward(75, 0.8);
        wheels.driveForward(25, 0.5, 4);
        if (isRed && propPosition == 1){
            wheels.driveLeft(10,0.5);
        }

        claw.openClawRight();
        claw.openClawLeft();
        opMode.sleep(500);

        wheels.driveBackword(10, 0.5);
        if (propPosition == 1) {
            if (!isRed) {
                wheels.driveLeft(60, 0.8,2);
            } else {
                wheels.driveRight(100, 0.8,2);
            }

        } else if (propPosition == 2) {
            if (!isRed) {
                wheels.driveLeft(80, 0.8,2);
            } else {
                wheels.driveRight(80, 0.8,2);
            }
        } else {
            if (!isRed) {
                wheels.driveLeft(100, 0.8,2);
            } else {
                wheels.driveRight(60, 0.8,2);
            }
        }
        wheels.driveForward(20, 0.8,2);
    }


    public void runRed() {
//        if (opMode.opModeInInit()){
//            imu = opMode.hardwareMap.get(IMU.class, "imu");
//            // Adjust the orientation parameters to match your robot
//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
//            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//            imu.initialize(parameters);
//
//            wheels = new Wheels(opMode, imu);
//            arm = new Arm(opMode); arm.initArm();
//            claw = new Claw(opMode); claw.initClaw();
//            elevator = new Elevator(opMode); elevator.initElevator();
//            intake = new Intake(opMode); intake.initIntake();
//
//            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
//            propProcessor = new ExampleProcessor(opMode.telemetry, 1);
//            portal = VisionPortal.easyCreateWithDefaults(
//                    webcamName, propProcessor);
//        }
//        while (opMode.opModeInInit() && !opMode.isStopRequested()){
//
//            propPosition = propProcessor.getPropPlacement();
//            opMode.telemetry.addData("prop",propPosition);
//            opMode.telemetry.update();
//            opMode.sleep(20);
//        }
//        wheels.driveForward(75, 0.5);
//        if( propPosition == 1 && !opMode.isStopRequested()){
//
//            wheels.rotateByEncoder(-90, 0.4);
//            wheels.driveLeft(7,0.4);
//            wheels.driveForward(30,0.5);
//            wheels.driveBackword(19,0.5);
//        }
//        else if(propPosition == 3 && !opMode.isStopRequested()){
//            wheels.rotateByEncoder(90,0.4);
//            wheels.driveForward(30,0.5);
//            wheels.driveBackword(15,0.5);
//
//        }
//        else {
//            wheels.driveForward(20,0.5);
//            wheels.driveBackword(18,0.5);
//        }
//
//        claw.closeClawRight();
//        arm.midArm();
//        opMode.sleep(1000);
//        arm.closeArm();
//        opMode.sleep(1000);
//        wheels.driveBackword(10,0.5);
        //isRed = true;
        runPeuple();
    }
}
