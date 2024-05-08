package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "Follow Example", group = "examples")
// @Disabled // Comment / uncomment to disable opmode
public class FollowExample extends LinearOpMode {

    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    private Wheels wheels;

    @Override
    public void runOpMode() throws InterruptedException {
        initProcessor();
        wheels = new Wheels(this, null);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> tags = tagProcessor.getDetections();

            for (AprilTagDetection tag : tags) {
                double xPos = tag.ftcPose.x;

                if (Math.abs(xPos) < .5)
                    wheels.driveRobotOriented(0, 0, xPos);
            }
        }

    }

    private void initProcessor() {
        WebcamName camDevice = hardwareMap.get(WebcamName.class, "Webcam 1");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(camDevice, tagProcessor);
    }
}
