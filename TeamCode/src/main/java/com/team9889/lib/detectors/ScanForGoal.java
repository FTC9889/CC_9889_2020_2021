package com.team9889.lib.detectors;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.subsystems.Robot;

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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by joshua9889 on 11/25/2019.
 */

@Config
public class ScanForGoal extends OpenCvPipeline {
//      RED VALUES
    public static double h1 = 100, h2 = 120;
    public static double s1 = 80, s2 = 255;
    public static double v1 = 0, v2 = 255;

//      BLUE VALUES
    public static double blueh1 = 0, blueh2 = 35;
    public static double blues1 = 123, blues2 = 181;
    public static double bluev1 = 0, bluev2 = 255;

    public static double size = 50;

    //Outputs
    private Mat cvResizeOutput = new Mat();
    private Mat cvBlurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>(), findContoursOutput2 = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>(), convexHullsOutput2 = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>(), filterContoursOutput2 = new ArrayList<MatOfPoint>();

    boolean debug = false;
    double upperPercentLimit = 0.55, lowerPercentLimit = 0.64;
    int threshold = 60;

    ElapsedTime fpsTimer = new ElapsedTime();

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
        fpsTimer.reset();

        // Step CV_resize0:
        Mat cvResizeSrc = input;
        double resizeFactor = 0.25;
        cvResize(cvResizeSrc, new Size(0, 0), resizeFactor, resizeFactor, Imgproc.INTER_LINEAR, cvResizeOutput);

        Mat cvBlur = cvResizeOutput;
        blur(cvBlur, ScanForRS.BlurType.GAUSSIAN, 2, cvBlurOutput);

        if (Robot.getInstance().blueGoal) {
            // Step HSV_Threshold0:
            Mat hsvThresholdInput = cvBlurOutput;
            double[] hsvThresholdHue = {blueh1, blueh2};
            double[] hsvThresholdSaturation = {blues1, blues2};
            double[] hsvThresholdValue = {bluev1, bluev2};
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
        } else {
            // Step HSV_Threshold0:
            Mat hsvThresholdInput = cvBlurOutput;
            double[] hsvThresholdHue = {h1, h2};
            double[] hsvThresholdSaturation = {s1, s2};
            double[] hsvThresholdValue = {v1, v2};
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
        }

        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, true, findContoursOutput);



        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
        double filterContoursMinArea = 0;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 0;
        double filterContoursMaxWidth = 1000;
        double filterContoursMinHeight = 0;
        double filterContoursMaxHeight = 1000;
        double[] filterContoursSolidity = {0.0, 100.0};
        double filterContoursMaxVertices = 1.0E7;
        double filterContoursMinVertices = 4;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 2;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        ArrayList<Double> areaArray = new ArrayList<>();
        for (int i = 0; i < filterContoursOutput.size(); i++) {
            double area = Imgproc.contourArea(filterContoursOutput.get(i));
            areaArray.add(area);
        }

        ArrayList<Double> sortedAreas = areaArray;
        Collections.sort(sortedAreas);

        Mat goal = new Mat();
        cvResizeOutput.copyTo(goal);

        if (sortedAreas.size() > 1) {
            int index1 = areaArray.indexOf(sortedAreas.get(sortedAreas.size() - 1));
            int index2 = areaArray.indexOf(sortedAreas.get(sortedAreas.size() - 2));

            Rect rect1 = Imgproc.boundingRect(filterContoursOutput.get(index1));
            Rect rect2 = Imgproc.boundingRect(filterContoursOutput.get(index2));

            if (rect1.x > rect2.x) {
                Rect rectTemp = rect2;
                rect2 = rect1;
                rect1 = rectTemp;
            }

            int xAverage = ((rect1.x + rect1.width) + (rect2.x)) / 2;
//            int yAverage = ((rect1.y + rect1.height / 2) + (rect2.y + rect2.height / 2)) / 2;

            Imgproc.line(goal, new Point(rect1.x + rect1.width, 0), new Point(rect1.x + rect1.width, goal.height()), new Scalar(0, 0, 255), 3);
            Imgproc.line(goal, new Point(rect2.x, 0), new Point(rect2.x, goal.height()), new Scalar(0, 0, 255), 3);

            Imgproc.rectangle(goal, rect1, new Scalar(0, 0, 0), -1);
            Imgproc.rectangle(goal, rect2, new Scalar(0, 0, 0), -1);
            Imgproc.line(goal, new Point(xAverage, 0), new Point(xAverage, goal.height()), new Scalar(0, 255, 0), 3);

            point = new Point(((double) (xAverage * 2) / (double) goal.width()) - 1, 0);
        }

        Log.v("FPS", "" + 1000.0 / fpsTimer.milliseconds());

        return goal;
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
            Log.i("Area", "" + area);
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
            Log.i("Ratio", "" + ratio);
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