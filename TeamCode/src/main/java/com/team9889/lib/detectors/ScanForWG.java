package com.team9889.lib.detectors;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by joshua9889 on 11/25/2019.
 */
public class ScanForWG extends OpenCvPipeline {
    private Mat cvResizeOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private Mat maskOutput = new Mat();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    private Point point = new Point(1e10, 1e10);

    public Point getPoint() {
        return point;
    }

    public ScanForWG() {

    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Mat cvResizeSrc = input;
        Size cvResizeDsize = new Size(0, 0);
        double cvResizeFx = 0.5;
        double cvResizeFy = 0.5;
        int cvResizeInterpolation = Imgproc.INTER_NEAREST;
        cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);

        // Step Blur0:
        Mat blurInput = cvResizeOutput;
        BlurType blurType = BlurType.get("Box Blur");
        double blurRadius = 0.0;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;

        double[] hsvThresholdHue = {10, 30};
        double[] hsvThresholdSaturation = {230, 255};
        double[] hsvThresholdValue = {0.0, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Mask0:
        Mat maskInput = blurOutput;
        Mat maskMask = hsvThresholdOutput;
        mask(maskInput, maskMask, maskOutput);

        // Step Find Contours
        Mat contourInput = new Mat();

        Imgproc.cvtColor(maskOutput, contourInput, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(contourInput, contourInput, Imgproc.COLOR_RGB2GRAY);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierachy = new Mat();
        Imgproc.findContours(contourInput, contours, hierachy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Step Filter_Contours0:
        List<MatOfPoint> filterContoursContours = contours;
        double filterContoursMinArea = 100.0;
        double filterContoursMinPerimeter = 25;
        double filterContoursMinWidth = 5;
        double filterContoursMaxWidth = 1000;
        double filterContoursMinHeight = 20;
        double filterContoursMaxHeight = 1000;
        double[] filterContoursSolidity = {0, 100.0};
        double filterContoursMaxVertices = 1000000;
        double filterContoursMinVertices = 0;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 1000;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        List<Moments> mu = new ArrayList<>();

        for (int i = 0; i < filterContoursOutput.size(); i++) {
            mu.add(Imgproc.moments(filterContoursOutput.get(i)));
        }

        List<Point> mc = new ArrayList<>();
        for (int i = 0; i < filterContoursOutput.size(); i++) {
            mc.add(new Point(mu.get(i).m10 / mu.get(i).m00 + 1e-5, mu.get(i).m01 / mu.get(i).m00 + 1e-5));
        }
        Imgproc.cvtColor(blurOutput, blurOutput, Imgproc.COLOR_HSV2RGB);

        for (int i = 0; i < filterContoursContours.size(); i++) {
            Imgproc.drawContours(blurOutput, filterContoursContours, i, new Scalar(0, 255, 0));
        }

        for (int i = 0; i < filterContoursOutput.size(); i++) {
            Imgproc.drawContours(blurOutput, filterContoursOutput, i, new Scalar(0, 0, 255));
            Imgproc.circle(blurOutput, mc.get(i), 4, new Scalar(0, 255, 0), -1);
            point = new Point((mc.get(i).x / ((double) blurOutput.width() / 2)) - 1, (mc.get(i).y / ((double) blurOutput.height() / 2)) - 1);
        }

//        return maskOutput;
        return blurOutput;
    }

    /**
     * Resizes an image.
     * @param src The image to resize.
     * @param dSize size to set the image.
     * @param fx scale factor along X axis.
     * @param fy scale factor along Y axis.
     * @param interpolation type of interpolation to use.
     * @param dst output image.
     */
    private void cvResize(Mat src, Size dSize, double fx, double fy, int interpolation,
                          Mat dst) {
        if (dSize==null) {
            dSize = new Size(0,0);
        }
        Imgproc.resize(src, dst, dSize, fx, fy, interpolation);
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @paramThe image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }
}
