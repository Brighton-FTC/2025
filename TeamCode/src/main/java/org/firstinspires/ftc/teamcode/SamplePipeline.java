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
    public static final double MIN_AREA = 5;
    public static final Size BLUR_SIZE = new Size(2, 3);

    private final Scalar lower, upper;
    private final List<MatOfPoint> contours = new ArrayList<>();

    private MatOfPoint largestContour;
    private final Mat hierarchy = new Mat();

    public SamplePipeline(Scalar lower, Scalar upper) {
        this.lower = lower;
        this.upper = upper;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat working = new Mat();
        Mat mask = new Mat();
        input.copyTo(working); // Copy original frame for processing

        // Convert to HSV and blur
        Imgproc.cvtColor(working, working, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(working, working, BLUR_SIZE);

        // Mask between bounds
        Core.inRange(working, lower, upper, mask);

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest contour
        largestContour = contours.stream()
                .max(Comparator.comparing(Imgproc::contourArea))
                .orElse(null);

        if (largestContour != null && Imgproc.contourArea(largestContour) < MIN_AREA) {
            largestContour = null;
        }

        // Return original frame so child pipeline can draw on it
        return input;
    }

    public MatOfPoint getLargestContour() {
        return largestContour;
    }
}

