package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import kotlin.math.pow

val goal = Pose(0.0, 141.0)
val shooterToGoalZSqrd = (46.0 - 13.5).pow(2.0)

object ShooterController {
    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    private val shooterLookupTable: Map<Double, ShotParameters> = mapOf(
//Distance      to ShotParameters(Dist,            Velocity,        Angle)                   // Origin (X, Y)
//        40.51   to ShotParameters(40.51, 950.0, 0.700),           // Pair(12, 120)
//        41.39   to ShotParameters(41.39, 965.0, 0.700),           // Pair(24, 132)
        45.53   to ShotParameters(45.53, 985.0, 0.680),           // Pair(24, 120) //checked
//        49.33   to ShotParameters(49.33, 980.0, 0.700),           // Pair(36, 132)
//        52.17   to ShotParameters(52.17, 1000.0, 0.600),          // Pair(24, 108)
//        52.85   to ShotParameters(52.85, 1000.0, 0.600),          // Pair(36, 120)
//        58.66   to ShotParameters(58.66, 1050.0, 0.600),          // Pair(48, 132)
//        58.66   to ShotParameters(58.66, 1060.0, 0.600),          // Pair(36, 108)
        61.65   to ShotParameters(61.65, 1060.0, 0.560),          // Pair(48, 120) //checked
//        66.16   to ShotParameters(66.16, 1125.0, 0.600),          // Pair(36, 96)
//        66.70   to ShotParameters(66.70, 1125.0, 0.600),          // Pair(48, 108)
//        68.83   to ShotParameters(68.83, 1140.0, 0.600),          // Pair(60, 132)
//        71.40   to ShotParameters(71.40, 1175.0, 0.600),          // Pair(60, 120)
        73.38   to ShotParameters(73.38, 1100.0, 0.580),          // Pair(48, 96) //checked
//        75.80   to ShotParameters(75.80, 1125.0, 0.600),          // Pair(60, 108)
//        79.51   to ShotParameters(79.51, 1200.0, 0.600),          // Pair(72, 132)
        81.74   to ShotParameters(81.74, 1180.0, 0.540),          // Pair(72, 120) // checked
//        81.30   to ShotParameters(81.30, 1205.0, 0.600),          // Pair(48, 84)
//        81.74   to ShotParameters(81.74, 1200.0, 0.560),          // Pair(60, 96)
//        85.61   to ShotParameters(85.61, 1100.0, 0.600),          // Pair(72, 108)
//        88.91   to ShotParameters(88.91, 1235.0, 0.575),          // Pair(60, 84)
//        90.52   to ShotParameters(90.52, 1260.0, 0.550),          // Pair(84, 132)
        90.91   to ShotParameters(90.91, 1230.0, 0.530),          // Pair(72, 96) //checked
//        92.48   to ShotParameters(92.48, 1225.0, 0.550),          // Pair(84, 120)
//        95.92   to ShotParameters(95.92, 1190.0, 0.575),          // Pair(84, 108)
//        97.04   to ShotParameters(97.04, 1150.0, 0.500),          // Pair(60, 72)
//        97.41   to ShotParameters(97.41, 1305.0, 0.550),          // Pair(72, 84)
//        100.68  to ShotParameters(100.68, 1375.0, 0.520),         // Pair(84, 96)
//        101.75  to ShotParameters(101.75, 1325.0, 0.550),         // Pair(96, 132)
        103.50  to ShotParameters(103.50, 1260.0, 0.500),         // Pair(96, 120) //checked
        104.89  to ShotParameters(104.89, 1255.0, 0.520),         // Pair(72, 72) //checked
//        106.59  to ShotParameters(106.59, 1300.0, 0.550),         // Pair(96, 108)
//        106.59  to ShotParameters(106.59, 1300.0, 0.525),         // Pair(84, 84)
        110.89  to ShotParameters(110.89, 1290.0, 0.500),         // Pair(96, 96) //checked
//        113.14  to ShotParameters(113.14, 1490.0, 0.525),         // Pair(108, 132)
//        113.14  to ShotParameters(113.14, 1325.0, 0.500),         // Pair(72, 60)
//        113.46  to ShotParameters(113.46, 1400.0, 0.500),         // Pair(84, 72)
//        114.72  to ShotParameters(114.72, 1425.0, 0.500),         // Pair(108, 120)
//        116.28  to ShotParameters(116.28, 1350.0, 0.500),         // Pair(96, 84)
//        117.51  to ShotParameters(117.51, 1360.0, 0.525),         // Pair(108, 108)
//        121.43  to ShotParameters(121.43, 1350.0, 0.600),         // Pair(108, 96)
//        124.65  to ShotParameters(124.65, 1550.0, 0.500),         // Pair(120, 132)
//        126.08  to ShotParameters(126.08, 1455.0, 0.500),         // Pair(120, 120)
//        128.63  to ShotParameters(128.63, 1395.0, 0.525),         // Pair(120, 108)
//        131.40  to ShotParameters(131.40, 1530.0, 0.525),         // Pair(72, 36)
//        135.44  to ShotParameters(135.44, 1530.0, 0.525),         // Pair(60, 24)
//        137.55  to ShotParameters(137.55, 1575.0, 0.500),         // Pair(132, 120)
        141.17  to ShotParameters(141.17, 1480.0, 0.500),         // Pair(72, 24) //checked
        141.43  to ShotParameters(141.43, 1480.0, 0.500),         // Pair(48, 12) //checked
//        145.94  to ShotParameters(145.94, 1550.0, 0.500),         // Pair(60, 12)
//        147.65  to ShotParameters(147.65, 1530.0, 0.525),         // Pair(84, 24)
//        149.11  to ShotParameters(149.11, 1550.0, 0.500),         // Pair(36, 0)
        151.27  to ShotParameters(151.27, 1560.0, 0.520),         // Pair(72, 12) //checked
//        152.45  to ShotParameters(152.45, 1550.0, 0.500),         // Pair(48, 0)
//        156.64  to ShotParameters(156.64, 1550.0, 0.500),         // Pair(60, 0)
//        157.33  to ShotParameters(157.33, 1550.0, 0.500),         // Pair(84, 12)
//        161.62  to ShotParameters(161.62, 1550.0, 0.500),         // Pair(72, 0)
        164.05  to ShotParameters(164.05, 1600.0, 0.520),         // Pair(96, 12) //checked
//        167.31  to ShotParameters(167.31, 1550.0, 0.500),         // Pair(84, 0)
//        173.65  to ShotParameters(173.65, 1550.0, 0.500),         // Pair(96, 0)
//        180.56  to ShotParameters(180.56, 1550.0, 0.500)          // Pair(108, 0)
    ).toSortedMap()

    private fun lerp(x: Double, x0: Double, x1: Double, y0: Double, y1: Double): Double {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    }


    /**
     * Finds the shot parameters for the distance closest to the target.
     * If the distance is within a certain threshold it returns the entry.
     */

    fun getShot(distance: Double): ShotParameters {
        val keys = shooterLookupTable.keys.sorted()
        val values = shooterLookupTable.values

        // Clamp to bounds
        if (distance <= keys.first()) {
            return values.first()
        }

        if (distance >= keys.last()) {
            return values.last()
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
