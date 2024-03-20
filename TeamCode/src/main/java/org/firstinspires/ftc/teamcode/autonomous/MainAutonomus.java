package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class MainAutonomus {
    boolean isRed;

    Wheels  wheels;
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
    public void run(){
        if (opMode.opModeInInit()){
            imu = opMode.hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            wheels = new Wheels(opMode, imu);
            arm = new Arm(opMode); arm.initArm();
            claw = new Claw(opMode); claw.initClaw();
            elevator = new Elevator(opMode); elevator.initElevator();
            intake = new Intake(opMode); intake.initIntake();

            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            propProcessor = new ExampleProcessor(opMode.telemetry, 2);
            portal = VisionPortal.easyCreateWithDefaults(
                    webcamName, propProcessor);
        }
        while (opMode.opModeInInit() && !opMode.isStopRequested()){

            propPosition = propProcessor.getPropPlacement();
            opMode.telemetry.addData("prop",propPosition);
            opMode.telemetry.update();
            opMode.sleep(20);
        }
        wheels.driveForward(75, 0.5);
        if( propPosition == 1 && !opMode.isStopRequested()){
            wheels.rotateByEncoder(-90,0.4);
            wheels.driveForward(20,0.5);
            wheels.driveBackword(5,0.5);
        }
        else if(propPosition == 3 && !opMode.isStopRequested()){
            wheels.rotateByEncoder(90, 0.4);
            wheels.driveForward(20,0.5);
            wheels.driveBackword(5,0.5);
        }



        arm.midArm();
        opMode.sleep(1000);
        arm.closeArm();
        opMode.sleep(1000);
        wheels.driveBackword(10,0.5);

}
}
