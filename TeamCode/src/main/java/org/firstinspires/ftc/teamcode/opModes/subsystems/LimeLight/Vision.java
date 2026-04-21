package org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class Vision extends LinearOpMode {

    // loading zone ROI
    double top = 0.35;
    double right = 0.65;
    double bottom = -0.65;
    // image constants
    final int IMAGE_WIDTH = 640;
    final int IMAGE_WIDTH_MIDDLE = IMAGE_WIDTH / 2;
    // loading zone
    final int ROI_WIDTH = (int) (IMAGE_WIDTH_MIDDLE * right * 2);
    final int ROI_FOURTH = ROI_WIDTH / 4;
    final int DIST_LEFT_ROI = (int) (IMAGE_WIDTH_MIDDLE * (1 - right));
    final int LEFT_BOUND = DIST_LEFT_ROI + ROI_FOURTH;
    final int MIDDLE_BOUND = DIST_LEFT_ROI + ROI_FOURTH + ROI_FOURTH;
    final int RIGHT_BOUND = DIST_LEFT_ROI + ROI_FOURTH + ROI_FOURTH + ROI_FOURTH;

    int region = 1; //0 left, 1 middle, 2 right for the indexing

    int region0AreaPurple = 0;
    int region1AreaPurple = 0;
    int region2AreaPurple = 0;
    int region3AreaPurple = 0;

    int region0AreaGreen = 0;
    int region1AreaGreen = 0;
    int region2AreaGreen = 0;
    int region3AreaGreen = 0;

    int biggestBlobAreaPurple = 0;
    int biggestBlobAreaGreen = 0;

    int purpleListLength = 0;
    int greenListLength = 8;

    WebcamName logiCam;

    ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-right, top, right, bottom))
            .setDrawContours(true) // show contours on the Stream Preview
            .setBoxFitColor(0) // disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 8)) // draw a circle
            .setBlurSize(5) // smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15) // expand blobs to fill any divots on the edges
            .setErodeSize(15) // shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();

    ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-right, top, right, bottom))
            .setDrawContours(true) // show contours on the Stream Preview
            .setBoxFitColor(0) // disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 8)) // draw a circle
            .setBlurSize(5) // smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15) // expand blobs to fill any divots on the edges
            .setErodeSize(15) // shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();


    public VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        // build portal
        portal = new VisionPortal.Builder()
                .addProcessors(purpleLocator, greenLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(logiCam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        while (opModeIsActive()) {

            // reset the areas per region
            region0AreaPurple = 0;
            region1AreaPurple = 0;
            region2AreaPurple = 0;
            region3AreaPurple = 0;
            region0AreaGreen = 0;
            region1AreaGreen = 0;
            region2AreaGreen = 0;
            region3AreaGreen = 0;

            biggestBlobAreaPurple = 0;
            biggestBlobAreaGreen = 0;

            // retrieve blob lists for this frame from locators
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();

            // if required for debugging e.g., not detecting anything
            purpleListLength = purpleBlobs.size();
            greenListLength = greenBlobs.size();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    300, 500_000, purpleBlobs); // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    300, 500_000, greenBlobs); // filter out very small blobs.

            // removed due to blobs being non circular detecting a group of artifacts
//            ColorBlobLocator Processor. Util.filterByCriteria(
//                    ColorBlobLocator Processor. BlobCriteria.BY_CIRCULARITY,
//                    0.6, 1, purpleBlobs);
//            filter out non-circular blobs.
//            ColorBlob Locator Processor. Util.filterByCriteria(
//                    ColorBlobLocator Processor.BlobCriteria.BY_CIRCULARITY,
//                    0.6, 1, greenBlobs);
// filter out non-circular blobs.

            for (ColorBlobLocatorProcessor.Blob blob : purpleBlobs) {
                Circle blobCircle = blob.getCircle();
                double circularity = blob.getCircularity();
                float radius = blobCircle.getRadius();
                int x = (int) blobCircle.getX();
                int y = (int) blobCircle.getY();
                int area = (int) (Math.PI * radius * radius);

                biggestBlobAreaPurple = Math.max(biggestBlobAreaPurple, area);

                if (x <= LEFT_BOUND) {
                    region0AreaPurple = Math.max(region0AreaPurple, area);
                } else if (x <= MIDDLE_BOUND) {
                    region1AreaPurple = Math.max(region1AreaPurple, area);
                } else if (x <= RIGHT_BOUND) {
                    region2AreaPurple = Math.max(region2AreaPurple, area);
                } else {
                    region3AreaPurple = Math.max(region3AreaPurple, area);
                }

            }

            int region0 = region0AreaPurple + region0AreaGreen;
            int region1 = region1AreaPurple + region1AreaGreen;
            int region2 = region2AreaPurple + region2AreaGreen;
            int region3 = region3AreaPurple + region3AreaGreen;

            region = 0;
            int max = region0;

            if (region1 > max) {
                region = 1;
                max = region1;
            }
            if (region2 > max) {
                region = 2;
                max = region2;
            }
            if (region3 > max) {
                region = 3;
            }

            if (purpleBlobs.isEmpty() && greenBlobs.isEmpty()) region = -1;

        }
    }

    public int getLoadingRegion() {
        return region;
    }
}