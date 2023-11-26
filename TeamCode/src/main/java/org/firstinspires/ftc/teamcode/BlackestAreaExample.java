package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This example uses OpenCV to determine the blackest area in the camera frame and
 * displays "right," "middle," or "left" based on that.
 */
@TeleOp
public class BlackestAreaExample extends LinearOpMode {
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.setPipeline(new BlackestAreaPipeline());
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Internal cam FPS", phoneCam.getFps());
            telemetry.update();

            sleep(100);
        }
    }

    class BlackestAreaPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            int height = input.rows();
            int width = input.cols();
            int leftZoneStart = width / 4;
            int middleZoneStart = width / 2;
            int rightZoneStart = 3 * width / 4;

            Mat leftZone = input.submat(height / 4, 3 * height / 4, 0, leftZoneStart);
            Mat middleZone = input.submat(height / 4, 3 * height / 4, leftZoneStart, middleZoneStart);
            Mat rightZone = input.submat(height / 4, 3 * height / 4, middleZoneStart, rightZoneStart);

            double leftAvgIntensity = Core.mean(leftZone).val[0];
            double middleAvgIntensity = Core.mean(middleZone).val[0];
            double rightAvgIntensity = Core.mean(rightZone).val[0];

            String position;
            if (leftAvgIntensity < middleAvgIntensity && leftAvgIntensity < rightAvgIntensity) {
                position = "left";
            } else if (middleAvgIntensity < rightAvgIntensity) {
                position = "middle";
            } else {
                position = "right";
            }

            telemetry.addData("Blackest Area", position);
            telemetry.update();

            Imgproc.rectangle(input, new Point(0, height / 4), new Point(leftZoneStart, 3 * height / 4), new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, new Point(leftZoneStart, height / 4), new Point(middleZoneStart, 3 * height / 4), new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(input, new Point(middleZoneStart, height / 4), new Point(rightZoneStart, 3 * height / 4), new Scalar(0, 0, 255), 2);

            return input;
        }
    }
}