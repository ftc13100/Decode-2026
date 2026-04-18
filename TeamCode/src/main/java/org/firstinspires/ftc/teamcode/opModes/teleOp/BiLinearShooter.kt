package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.control2.util.InterpolatingMap2D
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import kotlin.math.hypot

object BiLinearShooter {
    val goal = Pose(2.0, 132.0)

    data class ShotParameters(val velocity: Double, val angle: Double)

    private val xKeys = listOf(24.0, 48.0, 72.0, 96.0, 120.0)
    private val yKeys = listOf(21.97, 45.97, 69.97, 93.97, 117.97)

    private val shotData = listOf(
        // X, Y, Velocity, Angle, ToF
        listOf(24.0, 117.97, 1960.0, 0.8),
        listOf(24.0, 93.97, 1960.0, 0.8),
        listOf(48.0, 117.97, 1780.0, 0.550),
        listOf(48.0, 93.97, 1840.0, 0.750),
        listOf(48.0, 69.97, 1820.0, 0.650),
        listOf(72.0, 117.97, 1640.0, 0.600),
        listOf(72.0, 93.97, 1640.0, 0.600),
        listOf(72.0, 45.97, 1860.0, 0.700),
        listOf(96.0, 117.97, 1460.0, 0.300),
        listOf(72.0, 69.97, 1720.0, 0.700),
        listOf(96.0, 93.97, 1400.0, 0.350),
        listOf(96.0, 69.97, 1780.0, 0.700),
        listOf(120.0, 93.97, 1600.0, 0.700),
        listOf(72.0, 21.97, 2100.0, 0.800)
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
        val clampedX = x.coerceIn(xKeys.first(), xKeys.last())
        val clampedY = y.coerceIn(yKeys.first(), yKeys.last())
        val velocity = velocityMap[clampedX, clampedY]
        val angle = angleMap[clampedX, clampedY]
        return ShotParameters(velocity, angle)
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