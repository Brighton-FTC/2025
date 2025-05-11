package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public abstract class SamplePipeline extends OpenCvPipeline {
    public static final double MIN_AREA = 100;

    public static final Size BLUR_SIZE = new Size(7, 7);

    private final Scalar lower, upper;
    private final List<MatOfPoint> contours = new ArrayList<>();

    private MatOfPoint largestContour;

    // TODO: figure out what the hell this is for
    private final Mat hierarchy = new Mat();

    public SamplePipeline(Scalar lower, Scalar upper) {
        this.lower = lower;
        this.upper = upper;
    }

    @Override
    public Mat processFrame(Mat mat) {
        // convert the frame from RGB to HSV, which is better for color blob detection
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // blur image
        Imgproc.blur(mat, mat, BLUR_SIZE);

        // filter image for pixels in between lower and upper
        Core.inRange(mat, lower, upper, mat);

        // find all contours
        contours.clear();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour
        largestContour = contours.stream()
                .max(Comparator.comparing(Imgproc::contourArea))
                .orElse(null);

        if (largestContour != null && Imgproc.contourArea(largestContour) < MIN_AREA) {
            largestContour = null;
        }

        return mat;
    }

    public MatOfPoint getLargestContour() {
        return largestContour;
    }
}
