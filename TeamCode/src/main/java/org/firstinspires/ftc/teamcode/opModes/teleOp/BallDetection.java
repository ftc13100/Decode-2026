package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class BallDetection extends OpMode {

    OpenCvWebcam logiCam = null;

    // Ball detection data structures
    public List<BallCoordinates> purpleBalls = new ArrayList<>();
    public List<BallCoordinates> greenBalls = new ArrayList<>();

    // Robot position tracking
    public double robotX = 0;      // Robot's X position on field (inches)
    public double robotY = 0;      // Robot's Y position on field (inches)
    public double robotHeading = 0; // Robot's heading (degrees, 0 = facing right)

    // Camera calibration parameters
    private static final double CAMERA_FOV_X = 60.0;  // Horizontal field of view (degrees)
    private static final double CAMERA_FOV_Y = 45.0;  // Vertical field of view (degrees)
    private static final double CAMERA_HEIGHT_OFF_GROUND = 6.0; // Height above ground (inches)
    private static final double ESTIMATED_BALL_RADIUS = 1.0; // Estimated radius of ball (inches)

    // Camera resolution
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // Data class to store ball position
    public static class BallCoordinates {
        public double cameraX;      // X position in camera pixels (0-640)
        public double cameraY;      // Y position in camera pixels (0-360)
        public double fieldX;       // X position on field (inches)
        public double fieldY;       // Y position on field (inches)
        public double area;         // Ball size in pixels

        public BallCoordinates(double cameraX, double cameraY, double area) {
            this.cameraX = cameraX;
            this.cameraY = cameraY;
            this.area = area;
            this.fieldX = 0;
            this.fieldY = 0;
        }

        @Override
        public String toString() {
            return String.format("Ball(camera: %.1f,%.1f | field: %.1f,%.1f | area: %.1f)",
                cameraX, cameraY, fieldX, fieldY, area);
        }
    }

    /**
     * UPDATE THIS with your robot's odometry position
     * Call this every loop() with current robot position from odometry
     */
    public void updateRobotPosition(double x, double y, double heading) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
    }

    /**
     * Convert camera pixel coordinates to field coordinates
     * Uses robot position and heading to calculate where the ball actually is on the field
     */
    private void convertCameraToFieldCoordinates(BallCoordinates ball) {
        // Convert camera pixel to normalized coordinates (-1 to 1)
        double normalizedX = (ball.cameraX - CAMERA_WIDTH / 2.0) / (CAMERA_WIDTH / 2.0);
        double normalizedY = (ball.cameraY - CAMERA_HEIGHT / 2.0) / (CAMERA_HEIGHT / 2.0);

        // Convert normalized coordinates to angles (degrees)
        double angleX = normalizedX * (CAMERA_FOV_X / 2.0);  // Angle from center in X direction
        double angleY = normalizedY * (CAMERA_FOV_Y / 2.0);  // Angle from center in Y direction

        // Estimate distance using ball area (rough approximation)
        // Larger area = closer ball, smaller area = farther ball
        // This assumes we know the expected pixel area of the ball at a known distance
        double estimatedDistance = Math.sqrt((25000) / Math.max(ball.area, 10)); // Empirically tuned

        // Calculate ball position relative to camera (in camera's local coordinate system)
        // Camera points forward along robot's heading
        double relativeX = estimatedDistance * Math.cos(Math.toRadians(angleY)) * Math.sin(Math.toRadians(angleX));
        double relativeY = estimatedDistance * Math.cos(Math.toRadians(angleY)) * Math.cos(Math.toRadians(angleX));

        // Convert robot heading to radians
        double headingRad = Math.toRadians(robotHeading);

        // Rotate relative coordinates by robot heading to get field coordinates
        double fieldOffsetX = relativeX * Math.cos(headingRad) - relativeY * Math.sin(headingRad);
        double fieldOffsetY = relativeX * Math.sin(headingRad) + relativeY * Math.cos(headingRad);

        // Add robot position to get absolute field coordinates
        ball.fieldX = robotX + fieldOffsetX;
        ball.fieldY = robotY + fieldOffsetY;
    }

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "logiCam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        logiCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        logiCam.setPipeline(new autoPipeline());

        logiCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                logiCam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        }
        );
    }
    @Override
    public void loop() {
    }

    // Helper methods to access ball data from other programs

    /**
     * Get the closest purple ball to a specific point (in field coordinates)
     * @param centerX center X coordinate to measure from (field inches)
     * @param centerY center Y coordinate to measure from (field inches)
     * @return closest BallCoordinates or null if no balls found
     */
    public BallCoordinates getClosestPurpleBall(double centerX, double centerY) {
        if (purpleBalls.isEmpty()) return null;
        BallCoordinates closest = purpleBalls.get(0);
        double minDist = distance(closest.fieldX, closest.fieldY, centerX, centerY);

        for (BallCoordinates ball : purpleBalls) {
            double dist = distance(ball.fieldX, ball.fieldY, centerX, centerY);
            if (dist < minDist) {
                minDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Get the closest green ball to a specific point (in field coordinates)
     * @param centerX center X coordinate to measure from (field inches)
     * @param centerY center Y coordinate to measure from (field inches)
     * @return closest BallCoordinates or null if no balls found
     */
    public BallCoordinates getClosestGreenBall(double centerX, double centerY) {
        if (greenBalls.isEmpty()) return null;
        BallCoordinates closest = greenBalls.get(0);
        double minDist = distance(closest.fieldX, closest.fieldY, centerX, centerY);

        for (BallCoordinates ball : greenBalls) {
            double dist = distance(ball.fieldX, ball.fieldY, centerX, centerY);
            if (dist < minDist) {
                minDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Calculate distance between two points
     */
    private double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    class autoPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat purpleMask = new Mat();
        Mat greenMask = new Mat();
        Mat combinedMask = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(5, 5));

        // HSV Color ranges
        // Purple: H 125-155, S 100-255, V 100-255
        Scalar purpleLower = new Scalar(125, 100, 100);
        Scalar purpleUpper = new Scalar(155, 255, 255);

        // Green: H 35-85, S 100-255, V 100-255
        Scalar greenLower = new Scalar(35, 100, 100);
        Scalar greenUpper = new Scalar(85, 255, 255);

        // Minimum contour area to filter noise (adjust based on ball size)
        double minArea = 100;

        public Mat processFrame(Mat input) {
            // Convert to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks for purple and green balls
            Core.inRange(hsv, purpleLower, purpleUpper, purpleMask);
            Core.inRange(hsv, greenLower, greenUpper, greenMask);

            // Combine masks
            Core.bitwise_or(purpleMask, greenMask, combinedMask);

            // Morphological operations to clean up the mask
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Clear previous ball detections
            purpleBalls.clear();
            greenBalls.clear();

            // Process each contour
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                // Filter by minimum area
                if (area < minArea) continue;

                // Get bounding rectangle
                Rect boundRect = Imgproc.boundingRect(contour);

                // Calculate centroid
                org.opencv.moments.Moments moments = Imgproc.moments(contour);
                double centerX = moments.get_m10() / moments.get_m00();
                double centerY = moments.get_m01() / moments.get_m00();

                // Calculate circularity (4π * Area / Perimeter²)
                double perimeter = Imgproc.arcLength(new org.opencv.core.MatOfPoint2f(contour.toArray()), true);
                double circularity = 4 * Math.PI * area / (perimeter * perimeter);

                // Filter by circularity to ensure it's ball-shaped (0.7-1.0 is good for circles)
                if (circularity < 0.5) continue;

                // Check aspect ratio (should be close to 1.0 for circular objects)
                double aspectRatio = (double) boundRect.width / boundRect.height;
                if (aspectRatio < 0.6 || aspectRatio > 1.4) continue;

                // Determine color by checking which mask the contour belongs to
                double purplePixels = Core.countNonZero(new Mat(purpleMask, boundRect));
                double greenPixels = Core.countNonZero(new Mat(greenMask, boundRect));

                BallCoordinates ballCoord = new BallCoordinates(centerX, centerY, area);
p
                // Convert camera coordinates to field coordinates using robot position
                convertCameraToFieldCoordinates(ballCoord);

                if (purplePixels > greenPixels) {
                    purpleBalls.add(ballCoord);
                    // Draw purple circle
                    Imgproc.circle(input, new Point(centerX, centerY), (int)(perimeter / (2 * Math.PI)), new Scalar(200, 0, 200), 2);
                    Imgproc.putText(input, "P", new Point(centerX - 5, centerY - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(200, 0, 200), 2);
                } else {
                    greenBalls.add(ballCoord);
                    // Draw green circle
                    Imgproc.circle(input, new Point(centerX, centerY), (int)(perimeter / (2 * Math.PI)), new Scalar(0, 255, 0), 2);
                    Imgproc.putText(input, "G", new Point(centerX - 5, centerY - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                }

                // Draw center point
                Imgproc.circle(input, new Point(centerX, centerY), 3, new Scalar(255, 255, 0), -1);
            }

            // Update telemetry
            telemetry.addLine("=== BALL DETECTION ===");
            telemetry.addData("Purple balls found", purpleBalls.size());
            telemetry.addData("Green balls found", greenBalls.size());

            for (int i = 0; i < purpleBalls.size(); i++) {
                telemetry.addData("Purple #" + i, purpleBalls.get(i).toString());
            }

            for (int i = 0; i < greenBalls.size(); i++) {
                telemetry.addData("Green #" + i, greenBalls.get(i).toString());
            }

            telemetry.update();

            return input;
        }
    }
}

