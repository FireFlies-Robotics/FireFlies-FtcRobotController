package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Wheels;
import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "Blue Close Autonomous", group = "Autonomous")
public class BlueClose extends LinearOpMode {
    Arm arm;
    Claw claw;
    Elevator elevator;
    Intake intake;

    Wheels  wheels;
    IMU imu;

    VisionPortal portal;
    AprilTagProcessor processor;
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
        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(
                webcamName, processor
        );

        waitForStart();

        wheels.driveForward(65, .8);
        wheels.rotateByEncoder(-90, .5);
        arm.openArm();
        wheels.driveForward(60, .5);
        while (processor.getDetections().size() == 0) {
            wheels.driveRobotOriented(0, .3, 0);
        }

        while (!hastTag(processor.getDetections(), targetTagId)) {
            double strafeDirection = 0;
            boolean targetTagReached = false;
            boolean breakLoop = false;

            for (AprilTagDetection tag : processor.getDetections()) {
                if (!targetTagReached) {
                    if (tag.id == targetTagId) {
                        if (Math.abs(tag.ftcPose.x) < 1) {
                            breakLoop = true;
                        } else if (tag.ftcPose.x > 0) {
                            strafeDirection = .5;
                        } else {
                            strafeDirection = -.5;
                        }
                    } else if (tag.id > targetTagId) {
                        strafeDirection = -.5;
                    } else {
                        strafeDirection = .5;
                    }

                }

                if (breakLoop) break;
            }

            wheels.driveRobotOriented(strafeDirection, 0, 0);
        }

        boolean arivedToPosition = false;
        while (!arivedToPosition) {
            ArrayList<AprilTagDetection> detections = processor.getDetections();

            if (detections.size() > 0) {
                arivedToPosition = wheels.autoAdjust(detections, 20.32, targetTagId);
            } else {
                wheels.driveRobotOriented(0, 0, 0);
            }

            telemetry.addData("Loop", "autoadjust");
            telemetry.update();
        }
        telemetry.addData("Loop", "post-autoadjust");
        telemetry.update();
        claw.openClawLeft();
        claw.openClawRight();
        sleep(10);
        wheels.driveBackword(10, 0.5);
        wheels.driveLeft(65, 0.5);
        wheels.driveForward(10,1);
    }

    boolean hastTag(ArrayList<AprilTagDetection> detections, int targetTagId) {
        for (AprilTagDetection tag : detections) {
            if (tag.id == targetTagId) return true;
        }

        return false;
    }
}
