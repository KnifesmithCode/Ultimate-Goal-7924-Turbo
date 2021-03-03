package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RingsPipeline extends OpenCvPipeline {
    public enum RingQuantity {
        ZERO, ONE, FOUR
    }

    public RingQuantity qty;

    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat disp = new Mat();

    public static int RECT_X = 300;

    public static int RECT1_Y = 280;
    public static int RECT2_Y = 250;

    public static double THRESH = 100d;

    public static int HMIN = 0;
    public static int HMAX = 16;

    public static int SMIN = 100;
    public static int SMAX = 255;

    public static int VMIN = 0;
    public static int VMAX = 255;

    public RingQuantity getQty() {
        return qty != null ? qty : RingQuantity.ZERO;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(HMIN, SMIN, VMIN), new Scalar(HMAX, SMAX, VMAX), mask);

        Rect bottomRect = new Rect(RECT_X, RECT1_Y, 100, 10);
        double bottomAvg = Core.mean(new Mat(mask, bottomRect)).val[0];

        Rect topRect = new Rect(RECT_X, RECT2_Y, 100, 10);
        double topAvg = Core.mean(new Mat(mask, topRect)).val[0];

        Imgproc.cvtColor(mask, disp, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(disp, bottomRect, new Scalar(255, 0, 0), 1);
        Imgproc.rectangle(disp, topRect, new Scalar(0, 255, 0), 1);

        qty = RingQuantity.ZERO;

        if(bottomAvg >= THRESH) {
            qty = RingQuantity.ONE;
            if(topAvg >= THRESH) {
                qty = RingQuantity.FOUR;
            }
        }

        return disp;
    }
}
