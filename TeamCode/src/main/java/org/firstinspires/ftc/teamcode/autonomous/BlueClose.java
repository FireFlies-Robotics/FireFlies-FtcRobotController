package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Wheels;
import org.firstinspires.ftc.teamcode.eocv.ExampleProcessor;
import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "this one", group = "Autonomous")
public class BlueClose extends LinearOpMode {
    enum aliance{
        RED, BLUE;
    }
    aliance currentAliance = aliance.BLUE;

    Arm arm;
    Claw claw;
    Elevator elevator;
    Intake intake;

    Wheels  wheels;
    IMU imu;
    ExampleProcessor propProcessor;

    VisionPortal portal;
    AprilTagProcessor aprilProcessor;
    int propPosition = 3;

    ElapsedTime timeer =  new ElapsedTime();
    public void autonomusPurple(int propPosition){

        if (propPosition == 1){
            wheels.driveLeft(20,0.4);
        } else if (propPosition == 3) {
            wheels.driveRight(20,0.4);
        }
        wheels.driveForward(60, 0.4);
        arm.openArm();
        wheels.driveBackword(20,0.5);
        wheels.rotateByEncoder(-90,0.5);
        wheels.driveForward(66, 0.4);
    }

    public void autonomusYellow(aliance currentAliance, int targetTagId){
        if (currentAliance == aliance.RED) {
            targetTagId += 3;
        }
        wheels.driveForward(65, .8);

        if (aliance.BLUE == currentAliance) {
            wheels.rotateByEncoder(-90, .5);
        } else  {
            wheels.rotateByEncoder(90, .5);
        }

        if (opModeIsActive() && !isStopRequested()) {
            arm.openArm();
        }

        wheels.driveForward(60, .5);



        boolean arivedToPosition = false;
        while (!arivedToPosition && opModeIsActive() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = aprilProcessor.getDetections();

            if (detections.size() > 0) {
                arivedToPosition = wheels.autoAdjust(detections, 0, 20);
            } else if (timeer.seconds() < 20){
                wheels.driveRobotOriented(0, .3, 0);
                telemetry.addData("not seeing tags","");
                telemetry.update();
            }
            else {
                arivedToPosition = true;
            }
        }
        telemetry.addLine("Arrived to position");

        int minId = -1;
        double minRange = 0;

        for (AprilTagDetection tag : aprilProcessor.getDetections()) {
            double range = tag.ftcPose.range * 2.54;

            if (minId == -1) {
                minId = tag.id;
                minRange = range;
            } else if (minRange > range) {
                minId = tag.id;
                minRange = range;
            }
        }

//        wheels.driveRight(5*(targetTagId - minId), .6);
        wheels.driveRight(24*(targetTagId - minId), .6);
        telemetry.addData("strafe", (targetTagId - minId));
        telemetry.update();
        if (opModeIsActive() && !isStopRequested()){
            claw.openClawLeft();
            claw.openClawRight();
        }

        wheels.driveBackword(10, 0.5);
        wheels.driveLeft(65, 0.5);
        wheels.driveForward(10,1);
    }

    int targetTagId = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        wheels = new Wheels(this, imu);
        arm = new Arm(this); arm.initArm();
        claw = new Claw(this); claw.initClaw();
        elevator = new Elevator(this); elevator.initElevator();
        intake = new Intake(this); intake.initIntake();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilProcessor = AprilTagProcessor.easyCreateWithDefaults();
        propProcessor = new ExampleProcessor(telemetry, 2);

        portal = VisionPortal.easyCreateWithDefaults(
                webcamName, aprilProcessor);

        propPosition = propProcessor.getPropPlacement();
        telemetry.addData("Prop", propPosition);
        telemetry.update();

        waitForStart();
        timeer.reset();
        telemetry.addData("Prop", propPosition);
        telemetry.update();

        propPosition = 3;
        autonomusYellow(currentAliance, propPosition);

//        if (propPosition == 1){
//            telemetry.addData("moving",1);
//            telemetry.update();
//            sleep(1000);
//            wheels.driveRight(32, 0.5);
//            wheels.driveForward(60,0.5);
//        }
//        else if (propPosition == 3){
//            telemetry.addData("moving",3);
//            telemetry.update();
//            sleep(1000);
//            wheels.driveLeft(32,0.5);
//            wheels.driveForward(60,0.5);
//
//        }
//        else {
//            telemetry.addData("moving",2);
//            telemetry.update();
//            sleep(1000);
//            wheels.driveForward(90, 0.5);
//            wheels.driveBackword(13,0.5);
//        }
//        claw.closeClawLeft();
//        claw.closeClawRight();
//        arm.midArm();
//        sleep(300);
//        wheels.driveBackword(20,0.5);
//        arm.closeArm();
//        sleep(500);
//
    }
}
