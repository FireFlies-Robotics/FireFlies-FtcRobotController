package org.firstinspires.ftc.teamcode.eocv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "auto moto", group = "eocv")
public class ExamplePortal extends LinearOpMode {
    VisionPortal portal;
    ExampleProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {

        processor = new ExampleProcessor(telemetry);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

        waitForStart();
        while (opModeIsActive()) {
            //TODO Do stuff

            telemetry.update();
        }
    }
}
