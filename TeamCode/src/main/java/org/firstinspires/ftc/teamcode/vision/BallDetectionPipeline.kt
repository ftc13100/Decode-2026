package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.Collections

class BallDetectionPipeline(
    private val lowerHsv: Scalar,
    private val upperHsv: Scalar,
    private val minArea: Double = 200.0,
    private val debugCircleRadius: Int = 8,
) : OpenCvPipeline() {

    val balls: MutableList<Point> = Collections.synchronizedList(mutableListOf())

    @Volatile
    var frameWidth: Int = 0
        private set

    @Volatile
    var frameHeight: Int = 0
        private set

    private val rgb = Mat()
    private val hsv = Mat()
    private val mask = Mat()
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = ArrayList()
    private val debugColor = Scalar(0.0, 255.0, 0.0)

    override fun processFrame(input: Mat): Mat {
        frameWidth = input.width()
        frameHeight = input.height()

        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB)
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
        Core.inRange(hsv, lowerHsv, upperHsv, mask)

        for (contour in contours) {
            contour.release()
        }
        contours.clear()
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        synchronized(balls) {
            balls.clear()
            for (contour in contours) {
                val area = Imgproc.contourArea(contour)
                if (area < minArea) {
                    continue
                }

                val moments = Imgproc.moments(contour)
                if (moments.m00 == 0.0) {
                    continue
                }

                val center = Point(moments.m10 / moments.m00, moments.m01 / moments.m00)
                balls.add(center)
                Imgproc.circle(input, center, debugCircleRadius, debugColor, 2)
            }
        }

        return input
    }

    fun release() {
        rgb.release()
        hsv.release()
        mask.release()
        hierarchy.release()
        for (contour in contours) {
            contour.release()
        }
        contours.clear()
    }
}
