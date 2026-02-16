package org.firstinspires.ftc.teamcode.opModes.vision

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.vision.BallDetectionPipeline
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp(name = "FikerBallVisionThingymajigger", group = "Vision")
class FikerBallVisionThingymajigger : OpMode() {

    private lateinit var webcam: OpenCvWebcam
    private lateinit var pipeline: BallDetectionPipeline
    private lateinit var follower: Follower
    private var lastTarget: Pose? = null

    override fun init() {
        follower = Constants.createFollower(hardwareMap)

        pipeline = BallDetectionPipeline(
            lowerHsv = Scalar(20.0, 80.0, 80.0),
            upperHsv = Scalar(35.0, 255.0, 255.0),
            minArea = 250.0,
        )

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
        follower.update()

        val ballsSnapshot = synchronized(pipeline.balls) {
            pipeline.balls.map { Point(it.x, it.y) }
        }
        val firstBall = ballsSnapshot.firstOrNull()

        if (firstBall != null && !follower.isBusy) {
            val targetPose = pixelToField(firstBall, pipeline.frameWidth, pipeline.frameHeight)
            lastTarget = targetPose

            val path = Path(BezierLine(follower.pose, targetPose))
            path.setConstantHeadingInterpolation(targetPose.heading)
            follower.followPath(path)
        }

        telemetry.addData("Balls", ballsSnapshot.size)
        telemetry.addData(
            "BallCenters",
            ballsSnapshot.joinToString { "(%.1f, %.1f)".format(it.x, it.y) }
        )
        lastTarget?.let { telemetry.addData("TargetPose", "%.1f, %.1f", it.x, it.y) }
        telemetry.update()
    }

    override fun stop() {
        webcam.stopStreaming()
        webcam.closeCameraDevice()
        pipeline.release()
    }

    private fun pixelToField(ball: Point, frameWidth: Int, frameHeight: Int): Pose {
        if (frameWidth == 0 || frameHeight == 0) {
            return Pose(0.0, 0.0, 0.0)
        }

        val xNorm = (ball.x / frameWidth) - 0.5
        val yNorm = (ball.y / frameHeight) - 0.5

        val fieldX = xNorm * FIELD_WIDTH_IN
        val fieldY = -yNorm * FIELD_HEIGHT_IN

        return Pose(fieldX, fieldY, 0.0)
    }

    companion object {
        private const val FIELD_WIDTH_IN = 144.0
        private const val FIELD_HEIGHT_IN = 144.0
    }
}
