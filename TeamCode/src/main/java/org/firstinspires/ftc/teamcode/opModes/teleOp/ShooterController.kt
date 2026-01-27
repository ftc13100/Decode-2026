package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle


object ShooterController {
    val goal = Pose(6.0, 138.0)
    const val SHOOTER_TO_GOAL_Z_SQRD = 1139.0 // (46.0 - 12.25).pow(2.0)

    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    private val shooterLookupTable = mapOf(
   //Distance to ShotParameters(Dist,            Velocity,        Angle)            // Origin (X, Y)
        41.28 to ShotParameters(41.28, 985.0, 0.680),      // Pair(24, 120)
        47.75 to ShotParameters(47.75, 1020.0, 0.620),     // Pair(36, 120)
        56.07 to ShotParameters(56.07, 1000.0, 0.560),     // Pair(48, 120)
        60.99 to ShotParameters(60.99, 1020.0, 0.550),     // Pair(48, 108)
        67.71 to ShotParameters(67.71, 1065.0, 0.580),     // Pair(48, 96)
        75.74 to ShotParameters(75.74, 1080.0, 0.540),     // Pair(72, 120)
        79.45 to ShotParameters(79.45, 1150.0, 0.530),     // Pair(72, 108)
        84.71 to ShotParameters(84.71, 1180.0, 0.530),     // Pair(72, 96)
        97.37 to ShotParameters(97.37, 1200.0, 0.500),     // Pair(96, 120)
        98.83 to ShotParameters(98.83, 1200.0, 0.510),     // Pair(72, 72)
        104.50 to ShotParameters(104.50, 1290.0, 0.500),   // Pair(96, 96)
        135.68 to ShotParameters(135.68, 1460.0, 0.500),   // Pair(72, 24)
        136.73 to ShotParameters(136.73, 1460.0, 0.500),   // Pair(48, 12)
        145.90 to ShotParameters(145.90, 1510.0, 0.520),   // Pair(72, 12)
        158.22 to ShotParameters(158.22, 1560.0, 0.520),   // Pair(96, 12)
    ).toSortedMap()

    private fun lerp(x: Double, x0: Double, x1: Double, y0: Double, y1: Double): Double {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }

    /**
     * Finds the shot parameters for the distance closest to the target.
     * If the distance is within a certain threshold it returns the entry.
     */

    fun getShot(distance: Double): ShotParameters? {
        val keys = shooterLookupTable.keys.sorted()

        // Clamp to bounds
        if (distance !in keys.first()..keys.last()) {
            return null
        }

        var low = 0
        var high = keys.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midValue = keys[mid]

            when {
                distance < midValue -> high = mid - 1
                distance > midValue -> low = mid + 1
                else -> return shooterLookupTable[midValue]!!
            }
        }

        // At this point:
        // high < low
        // high is index of lower bound
        // low is index of upper bound

        val lowerKey = keys[high]
        val upperKey = keys[low]

        val lower = shooterLookupTable[lowerKey]!!
        val upper = shooterLookupTable[upperKey]!!

        return ShotParameters(
            distance = distance,
            velocity = lerp(distance, lowerKey, upperKey, lower.velocity, upper.velocity),
            angle = lerp(distance, lowerKey, upperKey, lower.angle, upper.angle)
        )
    }

    fun applyShot(params: ShotParameters) {
        // Set the hood angle
        ShooterAngle.targetPosition = params.angle

        // Schedule the movement and the flywheel spin
        CommandManager.scheduleCommand(
            ShooterAngle.update()
        )

        CommandManager.scheduleCommand(
            Shooter.spinAtSpeed(params.velocity)
        )
    }
}
