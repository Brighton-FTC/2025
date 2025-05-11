package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;

public class RedSamplePipeline extends SamplePipeline {
    public static final Scalar LOWER = new Scalar(140, 100, 160);
    public static final Scalar UPPER = new Scalar(240, 245, 240);

    public RedSamplePipeline() {
        super(LOWER, UPPER);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat output = super.processFrame(input);

        MatOfPoint contour = getLargestContour();
        if (contour != null) {
            Imgproc.drawContours(output, Collections.singletonList(contour), -1, new Scalar(255, 0, 0), 2);  // Blue color
        }

        return output;
    }
}
