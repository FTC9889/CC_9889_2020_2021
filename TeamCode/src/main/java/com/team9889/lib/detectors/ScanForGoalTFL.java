package com.team9889.lib.detectors;

import android.graphics.Bitmap;
import android.graphics.RectF;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.team9889.ftc2020.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.android.Utils;
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
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by joshua9889 on 11/25/2019.
 */

@Config
public class ScanForGoalTFL extends OpenCvPipeline {
    public Mat bitmap;

    private Point point = new Point(1e10, 1e10), pointInPixels = new Point(1e10, 1e10);

    public Point getPoint() {
        return point;
    }

    public Detection goal = null;

    public Pose2d odoPos = new Pose2d(0, 0, 0);

    ObjectDetector detector = null;
    public ScanForGoalTFL() {
        // Step 2: Initialize the detector object
        if (!Robot.getInstance().blue) {
            ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions.builder()
                    .setMaxResults(2)
                    .setScoreThreshold(0.8f)
                    .setNumThreads(2)
                    .build();

            try {
                detector = ObjectDetector.createFromFileAndOptions(
                        Robot.getInstance().hardwareMap.appContext, // the application context
                        "goal.tflite", // must be same as the filename in assets folder
                        options
                );
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions.builder()
                    .setMaxResults(2)
                    .setScoreThreshold(0.8f)
                    .setNumThreads(2)
                    .build();

            try {
                detector = ObjectDetector.createFromFileAndOptions(
                        Robot.getInstance().hardwareMap.appContext, // the application context
                        "BlueGoal.tflite", // must be same as the filename in assets folder
                        options
                );
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void blueGoal() {
        ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions.builder()
                .setMaxResults(2)
                .setScoreThreshold(0.8f)
                .setNumThreads(2)
                .build();

        try {
            detector = ObjectDetector.createFromFileAndOptions(
                    Robot.getInstance().hardwareMap.appContext, // the application context
                    "BlueGoal.tflite", // must be same as the filename in assets folder
                    options
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void redGoal() {
        ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions.builder()
                .setMaxResults(2)
                .setScoreThreshold(0.8f)
                .setNumThreads(2)
                .build();

        try {
            detector = ObjectDetector.createFromFileAndOptions(
                    Robot.getInstance().hardwareMap.appContext, // the application context
                    "goal.tflite", // must be same as the filename in assets folder
                    options
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    Mat cam = null;

    @Override
    public Mat processFrame(Mat inputMat) {
        cam = inputMat;

        if (goal != null) {
            RectF bb = goal.getBoundingBox();
            Imgproc.line(inputMat, new Point(bb.left, bb.top), new Point(bb.right, bb.top), new Scalar(0, 255, 0));
            Imgproc.line(inputMat, new Point(bb.right, bb.top), new Point(bb.right, bb.bottom), new Scalar(0, 255, 0));
            Imgproc.line(inputMat, new Point(bb.right, bb.bottom), new Point(bb.left, bb.bottom), new Scalar(0, 255, 0));
            Imgproc.line(inputMat, new Point(bb.left, bb.bottom), new Point(bb.left, bb.top), new Scalar(0, 255, 0));

            Imgproc.circle(inputMat, new Point(bb.centerX(), bb.centerY()), 5, new Scalar(0, 255, 255), -1);

            Imgproc.putText(inputMat, goal.getCategories().get(0).getLabel(),
                    new Point(bb.left, bb.bottom), 1, 3, new Scalar(0, 0, 255), 4);
        }

        return inputMat;
    }

    public void findGoal() {
        if (cam != null) {
            odoPos = Robot.getInstance().rr.getLocalizer().getPoseEstimate();
            odoPos = new Pose2d(odoPos.getX(), odoPos.getY(), Robot.getInstance().getMecanumDrive().gyroAngle.getTheda(AngleUnit.RADIANS));

            Bitmap input = Bitmap.createBitmap(cam.width(), cam.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(cam, input);

            // Step 1: create TFLite's TensorImage object
            TensorImage image = TensorImage.fromBitmap(input);

            // Step 3: feed given image to the model and print the detection result
            assert detector != null;
            List<Detection> results = detector.detect(image);

            double highestCon = 0;
            int goalIndex = -1;
            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).getCategories().get(0).getScore() > highestCon) {
                    Log.i("Score", "" + results.get(i).getCategories().get(0).getScore());
                    highestCon = results.get(i).getCategories().get(0).getScore();
                    goalIndex = i;
                }
            }

            if (goalIndex > -1) {
                Log.i("Result", results.get(goalIndex).toString());
                Log.i("Bounding Box", results.get(goalIndex).getBoundingBox().toString());
                Log.i("Class", results.get(goalIndex).getCategories().get(0).getLabel());

                goal = results.get(goalIndex);
            } else {
                goal = null;
            }
        }
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

        public static ScanForGoalTFL.BlurType get(String type) {
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