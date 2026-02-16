package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.Collections

class DualColorBallDetectionPipeline(
    private val lowerHsvColor1: Scalar,
    private val upperHsvColor1: Scalar,
    private val lowerHsvColor2: Scalar,
    private val upperHsvColor2: Scalar,
    private val minArea: Double = 200.0,
) : OpenCvPipeline() {

    val balls: MutableList<DetectedBall> = Collections.synchronizedList(mutableListOf())

    @Volatile
    var frameWidth: Int = 0
        private set

    @Volatile
    var frameHeight: Int = 0
        private set

    data class DetectedBall(val center: Point, val color: String, val area: Double)

    private val rgb = Mat()
    private val hsv = Mat()
    private val mask1 = Mat()
    private val mask2 = Mat()
    private val combinedMask = Mat()
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = ArrayList()

    override fun processFrame(input: Mat): Mat {
        frameWidth = input.width()
        frameHeight = input.height()

        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB)
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)

        Core.inRange(hsv, lowerHsvColor1, upperHsvColor1, mask1)
        Core.inRange(hsv, lowerHsvColor2, upperHsvColor2, mask2)
        Core.bitwise_or(mask1, mask2, combinedMask)

        for (contour in contours) {
            contour.release()
        }
        contours.clear()
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        synchronized(balls) {
            balls.clear()
            for (contour in contours) {
                val area = Imgproc.contourArea(contour)
                if (area < minArea) continue

                val moments = Imgproc.moments(contour)
                if (moments.m00 == 0.0) continue

                val center = Point(moments.m10 / moments.m00, moments.m01 / moments.m00)
                val colorLabel = if (mask1.get((center.y.toInt()).coerceIn(0, mask1.rows() - 1), (center.x.toInt()).coerceIn(0, mask1.cols() - 1))[0] > 0) "Green" else "Purple"
                val debugColor = if (colorLabel == "Green") Scalar(0.0, 255.0, 0.0) else Scalar(255.0, 0.0, 255.0)

                balls.add(DetectedBall(center, colorLabel, area))
                Imgproc.circle(input, center, 8, debugColor, 2)
            }
        }

        return input
    }

    fun release() {
        rgb.release(); hsv.release(); mask1.release(); mask2.release(); combinedMask.release(); hierarchy.release()
        for (contour in contours) contour.release()
        contours.clear()
    }
}

