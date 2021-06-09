package com.team9889.lib.detectors;

import com.acmerobotics.dashboard.config.Config;

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

@Config
public class ScanForGoal extends OpenCvPipeline {

    public static double h1 = 1, h2 = 90;
    public static double s1 = 0, s2 = 30, s3 = 80, s4 = 255;
    public static double v1 = 100, v2 = 255;
    public static double size = 50;

    //Outputs
    private Mat cvResizeOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>(), findContoursOutput2 = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>(), convexHullsOutput2 = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>(), filterContoursOutput2 = new ArrayList<MatOfPoint>();

    boolean debug = false;
    double upperPercentLimit = 0.55, lowerPercentLimit = 0.64;
    int threshold = 60;

    public Mat bitmap;

    private Point point = new Point(1e10, 1e10), pointInPixels = new Point(1e10, 1e10);

    public Point getPoint() {
        return point;
    }

    public Point getPointInPixels() {
        return pointInPixels;
    }

    public ScanForGoal() {

    }

    @Override
    public Mat processFrame(Mat input) {
        // Step CV_resize0:
        Mat cvResizeSrc = input;
        Size cvResizeDsize = new Size(0, 0);
        double cvResizeFx = 0.25;
        double cvResizeFy = 0.25;
        int cvResizeInterpolation = Imgproc.INTER_LINEAR;
        cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = cvResizeOutput;
        double[] hsvThresholdHue = {100, 120};
        double[] hsvThresholdSaturation = {s3, s4};
        double[] hsvThresholdValue = {0, 255};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step HSV_Threshold0:
//        Mat hsvThresholdInput = cvResizeOutput;
//        double[] hsvThresholdHueRed = {100, 120};
//        double[] hsvThresholdSaturationRed = {s3, s4};
//        double[] hsvThresholdValueRed = {0, 230};
//        hsvThreshold(hsvThresholdInput, hsvThresholdHueRed, hsvThresholdSaturationRed, hsvThresholdValueRed, hsvThresholdOutput);

        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = true;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        // Step Convex_Hulls0:
        ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
        convexHulls(convexHullsContours, convexHullsOutput);

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
        double filterContoursMinArea = 100;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 0;
        double filterContoursMaxWidth = 1000;
        double filterContoursMinHeight = 0;
        double filterContoursMaxHeight = 1000;
        double[] filterContoursSolidity = {0.0, 100.0};
        double filterContoursMaxVertices = 1.0E7;
        double filterContoursMinVertices = 4.0;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 1000;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        Mat fillPolyOut = new Mat();
        cvResizeOutput.copyTo(fillPolyOut);
        Imgproc.fillPoly(fillPolyOut, filterContoursOutput, new Scalar(0, 0, 0));

        // Step HSV_Threshold0:
        Mat mask = new Mat();
        double[] hsvThresholdHue2 = {0, 180};
        double[] hsvThresholdSaturation2 = {0, 255};
        double[] hsvThresholdValue2 = {0, 0};
        hsvThreshold(fillPolyOut, hsvThresholdHue2, hsvThresholdSaturation2, hsvThresholdValue2, mask);

        mask(cvResizeOutput, mask, fillPolyOut);

        // Step HSV_Threshold0:
        Mat mask2 = new Mat();
        double[] hsvThresholdHue3 = {h1, h2};
        double[] hsvThresholdSaturation3 = {s1, s2};
        double[] hsvThresholdValue3 = {v1, v2};
        hsvThreshold(fillPolyOut, hsvThresholdHue3, hsvThresholdSaturation3, hsvThresholdValue3, mask2);


        // Step Find_Contours0:
        Mat findContoursInput2 = mask2;
        boolean findContoursExternalOnly2 = true;
        findContours(findContoursInput2, findContoursExternalOnly2, findContoursOutput2);

        // Step Convex_Hulls0:
        ArrayList<MatOfPoint> convexHullsContours2 = findContoursOutput2;
        convexHulls(convexHullsContours2, convexHullsOutput2);

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours2 = convexHullsOutput2;
        double filterContoursMinArea2 = size;
        double filterContoursMinPerimeter2 = 0.0;
        double filterContoursMinWidth2 = 0;
        double filterContoursMaxWidth2 = 1000;
        double filterContoursMinHeight2 = 0;
        double filterContoursMaxHeight2 = 1000;
        double[] filterContoursSolidity2 = {0.0, 100.0};
        double filterContoursMaxVertices2 = 1.0E7;
        double filterContoursMinVertices2 = 4.0;
        double filterContoursMinRatio2 = 0;
        double filterContoursMaxRatio2 = 1000;
        filterContours(filterContoursContours2, filterContoursMinArea2, filterContoursMinPerimeter2, filterContoursMinWidth2, filterContoursMaxWidth2, filterContoursMinHeight2, filterContoursMaxHeight2, filterContoursSolidity2, filterContoursMaxVertices2, filterContoursMinVertices2, filterContoursMinRatio2, filterContoursMaxRatio2, filterContoursOutput2);

//        for (int i = 0; i < filterContoursOutput2.size(); i++) {
//            Imgproc.drawContours(cvResizeOutput, filterContoursOutput2, i, new Scalar(0, 255, 0), 2);
//        }

        List<Moments> mu = new ArrayList<>();

        for (int i = 0; i < filterContoursOutput2.size(); i++) {
            mu.add(Imgproc.moments(filterContoursOutput2.get(i)));
        }

        List<Point> mc = new ArrayList<>();
        for (int i = 0; i < filterContoursOutput2.size(); i++) {
            mc.add(new Point(mu.get(i).m10 / mu.get(i).m00 + 1e-5, mu.get(i).m01 / mu.get(i).m00 + 1e-5));
        }

        for (int i = 0; i < filterContoursOutput2.size(); i++) {
            Imgproc.drawContours(cvResizeOutput, filterContoursOutput2, i, new Scalar(0, 0, 255));
            Imgproc.circle(cvResizeOutput, mc.get(i), 4, new Scalar(0, 255, 0), -1);
        }

        if (mc.size() > 0) {
            Imgproc.circle(cvResizeOutput, mc.get(0), 2, new Scalar(0, 0, 255), -1);
            point = new Point((mc.get(0).x / ((double) cvResizeOutput.width() / 2)) - 1, (mc.get(0).y / ((double) cvResizeOutput.height() / 2)) - 1);
        } else {
            point = new Point (1e10, 1e10);
        }

        // TODO: Comment this out
        // Step CV_resize0:
        Mat cvResizeSrc2 = cvResizeOutput;
        Size cvResizeDsize2 = new Size(0, 0);
        double cvResizeFx2 = 4;
        double cvResizeFy2 = 4;
        int cvResizeInterpolation2 = Imgproc.INTER_LINEAR;
        cvResize(cvResizeSrc2, cvResizeDsize2, cvResizeFx2, cvResizeFy2, cvResizeInterpolation2, cvResizeOutput);

//        return hsvThresholdOutput;
        return input;
    }

    public Mat getImage () {
        return bitmap;
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

        public static ScanForGoal.BlurType get(String type) {
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

    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, ScanForRS.BlurType type, double doubleRadius,
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

    private void convexHulls(List<MatOfPoint> inputContours,
                             ArrayList<MatOfPoint> outputContours) {
        final MatOfInt hull = new MatOfInt();
        outputContours.clear();
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            outputContours.add(mopHull);
        }
    }

    private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
        Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
                new Scalar(red[1], green[1], blue[1]), out);
    }
}