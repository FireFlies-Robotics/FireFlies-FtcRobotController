package org.firstinspires.ftc.teamcode.eocv_testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Example VisionPortal OpMode")
public class ExampleVisionPortalOpMode extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTagProcessor;

    private AprilTagProcessor.Builder aprilTagProcessorBuilder;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        aprilTagProcessorBuilder.setDrawTagID(true);
        aprilTagProcessorBuilder.setDrawTagOutline(true);
        aprilTagProcessorBuilder.setDrawAxes(true);
        aprilTagProcessorBuilder.setDrawCubeProjection(true);

        aprilTagProcessor = aprilTagProcessorBuilder.build();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            for (AprilTagDetection tag : aprilTagProcessor.getDetections()) {
                //TODO Tag stuff
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
}