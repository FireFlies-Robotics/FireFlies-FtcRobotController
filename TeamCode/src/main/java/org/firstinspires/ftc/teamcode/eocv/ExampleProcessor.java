package org.firstinspires.ftc.teamcode.eocv;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ExampleProcessor implements VisionProcessor {

    Telemetry telemetry;

    ExampleProcessor() {super();}
    ExampleProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat leftCrop;
    Mat centerCrop;
    Mat rightCrop;
    double leftAvg;
    double centerAvg;
    double rightAvg;

    final int coi = 1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Size frameSize = frame.size();
        double third = frameSize.width / 3;
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2YCrCb);

        leftCrop = frame.submat(new Rect(0, 0, (int) third, (int) frameSize.height));
        centerCrop = frame.submat(new Rect((int) third, 0, (int) third, (int) frameSize.height));
        rightCrop = frame.submat(new Rect((int) (2 * third), 0, (int) third, (int) frameSize.height));

        Core.extractChannel(leftCrop, leftCrop, coi);
        Core.extractChannel(centerCrop, centerCrop, coi);
        Core.extractChannel(rightCrop, rightCrop, coi);
        Core.extractChannel(frame, frame, coi);

        leftAvg = Core.mean(leftCrop).val[0];
        centerAvg = Core.mean(centerCrop).val[0];
        rightAvg = Core.mean(rightCrop).val[0];

        double max = Math.max(Math.max(leftAvg, centerAvg), rightAvg);
        if (leftAvg == max) {
            telemetry.addLine("LEFT");
        } else if (centerAvg == max) {
            telemetry.addLine("CENTER");
        } else {
            telemetry.addLine("RIGHT");
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        float third = onscreenWidth/3;
        canvas.drawLine(third, 0, third, onscreenHeight, new Paint());
        canvas.drawLine(2 * third, 0, 2 * third, onscreenHeight, new Paint());
    }
}
