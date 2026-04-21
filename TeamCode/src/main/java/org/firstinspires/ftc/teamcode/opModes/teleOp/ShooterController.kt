package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle


object ShooterController {
    val goalBlue = Pose(6.0, 138.0)
    val goalRed = Pose(138.0, 138.0)
    const val SHOOTER_TO_GOAL_Z_SQRD = 1056.25 // (46.0 - 13.5).pow(2.0)

    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)
// note: all of these points and distances were created considered robot facing the goal with minimal turret angle
// basically if goal was 24,24 and robot was at 24,20, distance of shooter was actually 24,18 if shooter offset was 2 in behind
// does this mean you can just subtract 2 from distance? i don"t think so i can't fully think abt it rn gn
    private val shooterLookupTable = mapOf(
   //Distance to ShotParameters(Dist,            Velocity,        Angle)            // Origin (X, Y)
    44.68 to ShotParameters(44.68, 985.0, 0.680),      // Pair(24, 120)
    51.15 to ShotParameters(51.15, 1020.0, 0.620),     // Pair(36, 120)
    59.47 to ShotParameters(59.47, 1000.0, 0.560),     // Pair(48, 120)
    64.39 to ShotParameters(64.39, 1020.0, 0.550),     // Pair(48, 108)
    71.11 to ShotParameters(71.11, 1065.0, 0.580),     // Pair(48, 96)
    79.14 to ShotParameters(79.14, 1080.0, 0.540),     // Pair(72, 120)
    82.85 to ShotParameters(82.85, 1150.0, 0.530),     // Pair(72, 108)
    88.11 to ShotParameters(88.11, 1180.0, 0.530),     // Pair(72, 96)
    93.07 to ShotParameters(93.07, 1230.0, 0.520),     // Pair(84,108)
    100.77 to ShotParameters(100.77, 1230.0, 0.510),   // Pair(96, 120)
    102.23 to ShotParameters(102.23, 1200.0, 0.510),   // Pair(72, 72)
    107.90 to ShotParameters(107.90, 1290.0, 0.500),   // Pair(96, 96)
    139.08 to ShotParameters(139.08, 1440.0, 0.500),   // Pair(72, 24)
    140.13 to ShotParameters(140.13, 1460.0, 0.500),   // Pair(48, 12)
    149.30 to ShotParameters(149.30, 1500.0, 0.520),   // Pair(72, 12)
    155.11 to ShotParameters(155.11, 1530.0, 0.500),   // Pair(84, 12)
    161.62 to ShotParameters(161.62, 1560.0, 0.520),   // Pair(96, 12)
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
