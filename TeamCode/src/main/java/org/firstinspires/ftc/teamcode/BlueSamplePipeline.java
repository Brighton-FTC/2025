package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;

public class BlueSamplePipeline extends SamplePipeline {
    public static final Scalar LOWER = new Scalar(100, 100, 50);
    public static final Scalar UPPER = new Scalar(130, 255, 255);


    public BlueSamplePipeline() {
        super(LOWER, UPPER);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat output = super.processFrame(input);

        MatOfPoint contour = getLargestContour();
        if (contour != null) {
            Imgproc.drawContours(output, Collections.singletonList(contour), -1, new Scalar(255, 0, 0), 2);  // Magenta = visible on most backgrounds
        }
        Imgproc.circle(output, new Point((double) output.cols() /2, (double) output.rows() /2), 15, new Scalar(0, 255, 0), 3);

        return output;
    }

}
