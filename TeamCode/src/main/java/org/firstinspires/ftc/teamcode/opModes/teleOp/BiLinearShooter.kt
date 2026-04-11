package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import dev.nextftc.control2.util.InterpolatingMap2D

object BiLinearShooter {
    val goal = Pose(6.0, 138.0)

    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    private val xKeys = listOf(24.0, 36.0, 48.0, 72.0, 96.0, 108.0)
    private val yKeys = listOf(12.0, 24.0, 48.0, 72.0, 96.0, 108.0, 120.0)

    private val shotData = listOf(
        // X, Y, Velocity, Angle, ToF
        listOf(24.0, 120.0, 985.0, 0.104),
        listOf(36.0, 120.0, 1020.0, 0.145),
        listOf(48.0, 120.0, 1000.0, 0.186),
        listOf(48.0, 108.0, 1020.0, 0.193),
        listOf(48.0, 96.0, 1065.0, 0.173),
        listOf(72.0, 120.0, 1080.0, 0.200),
        listOf(72.0, 108.0, 1150.0, 0.207),
        listOf(72.0, 96.0, 1180.0, 0.207),
        listOf(96.0, 120.0, 1200.0, 0.225),
        listOf(72.0, 72.0, 1200.0, 0.218),
        listOf(96.0, 96.0, 1290.0, 0.225),
        listOf(72.0, 24.0, 1460.0, 0.225),
        listOf(48.0, 12.0, 1460.0, 0.225),
        listOf(72.0, 12.0, 1510.0, 0.212),
        listOf(96.0, 12.0, 1560.0, 0.212)
    )

    private val velocityGrid = List(yKeys.size) { yIndex ->
        List(xKeys.size) { xIndex ->
            shotData.find { it[0] == xKeys[xIndex] && it[1] == yKeys[yIndex] }?.get(2) ?: 0.0
        }
    }

    private val angleGrid = List(yKeys.size) { yIndex ->
        List(xKeys.size) { xIndex ->
            shotData.find { it[0] == xKeys[xIndex] && it[1] == yKeys[yIndex] }?.get(3) ?: 0.0
        }
    }

    private val velocityMap = InterpolatingMap2D.bilinear(xKeys, yKeys, velocityGrid)
    private val angleMap = InterpolatingMap2D.bilinear(xKeys, yKeys, angleGrid)

    /**
     * Get shot parameters using bilinear interpolation based on X and Y position.
     */
    fun getShot(x: Double, y: Double): ShotParameters {
        val velocity = velocityMap[x, y]
        val angle = angleMap[x, y]
        val distance = Math.hypot(goal.x - x, goal.y - y)
        return ShotParameters(distance, velocity, angle)
    }

    /**
     * Apply a shot by setting the hood angle and flywheel speed.
     */
    fun applyShot(params: ShotParameters) {
        ShooterAngle.targetPosition = params.angle
        CommandManager.scheduleCommand(ShooterAngle.update())
        CommandManager.scheduleCommand(Shooter.spinAtSpeed(params.velocity))
    }
}