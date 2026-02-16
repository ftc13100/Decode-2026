package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous
object opencv : Subsystem {
    var logiCam: OpenCvWebcam? = null

    fun init() {
        val webcamName = hardwareMap.get<WebcamName?>(WebcamName::class.java, "logiCam")
        val cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        logiCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        logiCam!!.setPipeline(autoPipeline())

        logiCam!!.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                logiCam!!.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
            }
        }
        )
    }

     fun loop() {
    }

    internal class autoPipeline : OpenCvPipeline() {
        var YCbCR: Mat = Mat()
        var leftCrop: Mat? = null
        var rightCrop: Mat? = null
        var leftavgfin: Double = 0.0
        var rightavgfin: Double = 0.0
        var outPut: Mat = Mat()
        var rectColor: Scalar = Scalar(255.0, 0.0, 0.0)

        override fun processFrame(input: Mat): Mat? {
            Imgproc.cvtColor(input, YCbCR, Imgproc.COLOR_RGB2YCrCb)
            telemetry.addLine("pipeline running")

            val leftRect = Rect(1, 1, 319, 359)
            val rightRect = Rect(320, 1, 319, 359)


            input.copyTo(outPut)
            Imgproc.rectangle(outPut, leftRect, rectColor, 2)
            Imgproc.rectangle(outPut, rightRect, rectColor, 2)

            leftCrop = YCbCR.submat(leftRect)
            rightCrop = YCbCR.submat(rightRect)

            Core.extractChannel(leftCrop, leftCrop, 2)
            Core.extractChannel(rightCrop, rightCrop, 2)


            val leftavg = Core.mean(leftCrop)
            val rightavg = Core.mean(rightCrop)

            leftavgfin = leftavg.`val`[0]
            rightavgfin = rightavg.`val`[0]

            if (leftavgfin > rightavgfin) {
                telemetry.addLine("Right")
            } else if (rightavgfin > leftavgfin) {
                telemetry.addLine("Left")
            } else {
                telemetry.addLine("nothing found")
            }


            return (outPut)
        }
    }
}