package org.firstinspires.ftc.teamcode.opModes.vision

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "HSVColorCalibrator", group = "Vision")
class HSVColorCalibrator : OpMode() {

    private lateinit var webcam: OpenCvWebcam
    private lateinit var pipeline: CalibrationPipeline

    private var sampleRadius = 15
    private var sampleX = 320
    private var sampleY = 240

    override fun init() {
        pipeline = CalibrationPipeline()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val webcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)
        webcam.setPipeline(pipeline)
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                telemetry.addData("Camera", "Open error: %d", errorCode)
            }
        })
    }

    override fun loop() {
        // Adjust sample position with dpad
        if (gamepad1.dpad_up) sampleY = max(sampleRadius, sampleY - 5)
        if (gamepad1.dpad_down) sampleY = min(480 - sampleRadius, sampleY + 5)
        if (gamepad1.dpad_left) sampleX = max(sampleRadius, sampleX - 5)
        if (gamepad1.dpad_right) sampleX = min(640 - sampleRadius, sampleX + 5)

        // Adjust radius with bumpers
        if (gamepad1.left_bumper) sampleRadius = max(5, sampleRadius - 2)
        if (gamepad1.right_bumper) sampleRadius = min(50, sampleRadius + 2)

        pipeline.setSamplePoint(sampleX, sampleY, sampleRadius)

        telemetry.addLine("=== HSV Color Calibrator ===")
        telemetry.addLine("Position your ball over the crosshair")
        telemetry.addLine("Use D-Pad to move, Bumpers to resize circle")
        telemetry.addLine()

        telemetry.addData("Sample X", sampleX)
        telemetry.addData("Sample Y", sampleY)
        telemetry.addData("Radius", sampleRadius)
        telemetry.addLine()

        val meanHsv = pipeline.meanHsv
        if (meanHsv != null) {
            val h = meanHsv[0]
            val s = meanHsv[1]
            val v = meanHsv[2]

            telemetry.addData("Mean H", "%.1f (0-180 range)", h)
            telemetry.addData("Mean S", "%.1f (0-255)", s)
            telemetry.addData("Mean V", "%.1f (0-255)", v)
            telemetry.addLine()

            telemetry.addLine("Copy this into BallDetectionPipeline:")
            telemetry.addLine("lowerHsv = Scalar(${(h - 10).coerceAtLeast(0.0)}, 50.0, 50.0)")
            telemetry.addLine("upperHsv = Scalar(${(h + 10).coerceAtMost(180.0)}, 255.0, 255.0)")
        }

        telemetry.update()
    }

    override fun stop() {
        webcam.stopStreaming()
        webcam.closeCameraDevice()
        pipeline.release()
    }

    private inner class CalibrationPipeline : OpenCvPipeline() {
        private val hsv = Mat()
        private val sampleMask = Mat()

        @Volatile
        var meanHsv: DoubleArray? = null
            private set

        private var sampleX = 0
        private var sampleY = 0
        private var sampleRadius = 15

        fun setSamplePoint(x: Int, y: Int, radius: Int) {
            this.sampleX = x
            this.sampleY = y
            this.sampleRadius = radius
        }

        override fun processFrame(input: Mat): Mat {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV)

            sampleMask.release()
            sampleMask.create(hsv.rows(), hsv.cols(), org.opencv.core.CvType.CV_8U)
            sampleMask.setTo(Scalar(0.0))

            Imgproc.circle(sampleMask, Point(sampleX.toDouble(), sampleY.toDouble()), sampleRadius, Scalar(255.0), -1)

            val mean = Core.mean(hsv, sampleMask)
            meanHsv = doubleArrayOf(mean.`val`[0], mean.`val`[1], mean.`val`[2])

            // Draw crosshair on output
            Imgproc.circle(input, Point(sampleX.toDouble(), sampleY.toDouble()), sampleRadius, Scalar(0.0, 255.0, 255.0), 2)
            Imgproc.line(input, Point((sampleX - 20).toDouble(), sampleY.toDouble()), Point((sampleX + 20).toDouble(), sampleY.toDouble()), Scalar(0.0, 255.0, 255.0), 2)
            Imgproc.line(input, Point(sampleX.toDouble(), (sampleY - 20).toDouble()), Point(sampleX.toDouble(), (sampleY + 20).toDouble()), Scalar(0.0, 255.0, 255.0), 2)

            return input
        }

        fun release() {
            hsv.release()
            sampleMask.release()
        }
    }
}

