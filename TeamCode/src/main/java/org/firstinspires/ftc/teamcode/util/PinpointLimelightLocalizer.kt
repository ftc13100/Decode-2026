package org.firstinspires.ftc.teamcode.util

import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.simple.SimpleMatrix
import kotlin.math.PI

class PinpointLimelightLocalizer(
    map: HardwareMap,
    constants: PinpointConstants,
    private val limelight: Limelight3A
) : PinpointLocalizer(map, constants) {
    // EKF state: [x, y, heading]
    private val ekf: EKF

    // last update time for dt
    private var lastUpdateTime = System.nanoTime()
    private var lastOdomPose: Pose = super.getPose()

    init {
        // Initial state: from odometry
        val odom = super.getPose()
        val x0 = SimpleMatrix(
            arrayOf(
                doubleArrayOf(odom.x),
                doubleArrayOf(odom.y),
                doubleArrayOf(odom.heading)
            )
        )

        val P0 = SimpleMatrix.identity(3) * 0.1 // initial covariance
        val Q = SimpleMatrix.identity(3) * 0.01 // process noise
        val R = SimpleMatrix.identity(3) * 0.05 // measurement noise

        ekf = EKF(x0, P0, Q, R)

        // Process model: x_next = x + dx_odom, y_next = y + dy_odom, heading_next = heading + dheading
        ekf.f = { x, u ->
            // u: odometry delta since last update [dx, dy, dheading]
            x + u
        }

        // Jacobian of process: identity
        ekf.fJacobian = { _, _ -> SimpleMatrix.identity(3) }

        // Measurement: full pose
        ekf.h = { x -> x.copy() }
        ekf.hJacobian = { _ -> SimpleMatrix.identity(3) }
    }

    override fun update() {
        super.update() // update odometry

        // Compute odometry delta since last EKF update
        val odomDelta = computeOdomDelta()
        ekf.predict(odomDelta)

        // Get Limelight measurement
        val result = limelight.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            if (fiducials.isNotEmpty()) {
                val tagPose = fiducials[0].robotPoseFieldSpace
                if (tagPose != null) {
                    val visionPose = SimpleMatrix(arrayOf(
                        doubleArrayOf(tagPose.position.x),
                        doubleArrayOf(tagPose.position.y),
                        doubleArrayOf(Math.toRadians(tagPose.orientation.yaw))
                    ))

                    ekf.update(visionPose)
                }
            }
        }

        // Update robot pose from EKF
        val x = ekf.x
        super.setPose(Pose(x[0], x[1], x[2]))
    }

    private fun computeOdomDelta(): SimpleMatrix {
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastUpdateTime) / 1e9
        lastUpdateTime = currentTime

        // Get current odometry velocities
        val v = super.velocity // assume this is a Pose-like object with vx, vy, heading rate
        val vx = v.x          // velocity in field x
        val vy = v.y          // velocity in field y
        val omega = v.heading // angular velocity (radians/sec)

        // Displacement = velocity * dt
        // If velocities are in robot frame, rotate to field frame:
        val heading = lastOdomPose.heading
        val dx = vx * kotlin.math.cos(heading) - vy * kotlin.math.sin(heading)
        val dy = vx * kotlin.math.sin(heading) + vy * kotlin.math.cos(heading)
        val dheading = omega * dt

        // Update last pose for next delta
        lastOdomPose = super.getPose()

        return SimpleMatrix(arrayOf(
            doubleArrayOf(dx * dt),
            doubleArrayOf(dy * dt),
            doubleArrayOf(dheading)
        ))
    }

    private fun angleWrap(angle: Double): Double {
        var a = angle
        while (a > PI) a -= 2 * PI
        while (a < -PI) a += 2 * PI
        return a
    }
}