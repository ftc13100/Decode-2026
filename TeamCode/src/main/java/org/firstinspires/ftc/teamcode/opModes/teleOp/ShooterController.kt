package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import kotlin.math.pow

val goal = Pose(2.0, 142.0)
val shooterToGoalZSqrd = (46.0 - 13.5).pow(2.0)

object ShooterController {
    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    private val shooterLookupTable: Map<Double, ShotParameters> = mapOf(
//Distance      to ShotParameters(Dist,            Velocity,        Angle)                   // Origin (X, Y)
        40.50   to ShotParameters(40.50, 925.0, 0.700),           // Pair(12, 120)
        40.50   to ShotParameters(40.50, 950.0, 0.700),           // Pair(24, 132)
        44.99   to ShotParameters(44.99, 950.0, 0.700),           // Pair(24, 120)
        48.09   to ShotParameters(48.09, 975.0, 0.700),           // Pair(36, 132)
        51.93   to ShotParameters(51.93, 1000.0, 0.600),          // Pair(24, 108)
        51.93   to ShotParameters(51.93, 975.0, 0.650),           // Pair(36, 120)
        57.20   to ShotParameters(57.20, 1000.0, 0.600),          // Pair(48, 132)
        58.04   to ShotParameters(58.04, 1060.0, 0.600),          // Pair(36, 108)
        60.47   to ShotParameters(60.47, 1000.0, 0.600),          // Pair(48, 120)
        65.79   to ShotParameters(65.79, 1125.0, 0.600),          // Pair(36, 96)
        65.79   to ShotParameters(65.79, 1120.0, 0.600),          // Pair(48, 108)
        67.23   to ShotParameters(67.23, 1100.0, 0.600),          // Pair(60, 132)
        70.03   to ShotParameters(70.03, 1100.0, 0.600),          // Pair(60, 120)
        72.72   to ShotParameters(72.72, 1190.0, 0.580),          // Pair(48, 96)
        74.67   to ShotParameters(74.67, 1125.0, 0.600),          // Pair(60, 108)
        77.82   to ShotParameters(77.82, 1200.0, 0.600),          // Pair(72, 132)
        80.25   to ShotParameters(80.25, 1200.0, 0.600),          // Pair(72, 120)
        80.85   to ShotParameters(80.85, 1305.0, 0.600),          // Pair(48, 84)
        80.85   to ShotParameters(80.85, 1300.0, 0.560),          // Pair(60, 96)
        84.33   to ShotParameters(84.33, 1200.0, 0.600),          // Pair(72, 108)
        88.23   to ShotParameters(88.23, 1335.0, 0.575),          // Pair(60, 84)
        88.77   to ShotParameters(88.77, 1362.0, 0.550),          // Pair(84, 132)
        89.85   to ShotParameters(89.85, 1350.0, 0.540),          // Pair(72, 96)
        90.91   to ShotParameters(90.91, 1325.0, 0.550),          // Pair(84, 120)
        94.53   to ShotParameters(94.53, 1288.0, 0.575),          // Pair(84, 108)
        96.54   to ShotParameters(96.54, 1250.0, 0.500),          // Pair(60, 72)
        96.54   to ShotParameters(96.54, 1405.0, 0.550),          // Pair(72, 84)
        99.48   to ShotParameters(99.48, 1375.0, 0.520),          // Pair(84, 96)
        99.96   to ShotParameters(99.96, 1525.0, 0.550),          // Pair(96, 132)
        101.86  to ShotParameters(101.86, 1440.0, 0.500),         // Pair(96, 120)
        104.19  to ShotParameters(104.19, 1380.0, 0.520),         // Pair(72, 72)
        105.11  to ShotParameters(105.11, 1375.0, 0.550),         // Pair(96, 108)
        105.57  to ShotParameters(105.57, 1400.0, 0.525),         // Pair(84, 84)
        109.58  to ShotParameters(109.58, 1400.0, 0.500),         // Pair(96, 96)
        111.32  to ShotParameters(111.32, 1588.0, 0.525),         // Pair(108, 132)
        112.61  to ShotParameters(112.61, 1425.0, 0.500),         // Pair(72, 60)
        112.61  to ShotParameters(112.61, 1500.0, 0.500),         // Pair(84, 72)
        113.03  to ShotParameters(113.03, 1525.0, 0.500),         // Pair(108, 120)
        115.14  to ShotParameters(115.14, 1450.0, 0.500),         // Pair(96, 84)
        115.97  to ShotParameters(115.97, 1462.0, 0.525),         // Pair(108, 108)
        120.03  to ShotParameters(120.03, 1450.0, 0.600),         // Pair(108, 96)
        122.80  to ShotParameters(122.80, 1650.0, 0.500),         // Pair(120, 132)
        124.36  to ShotParameters(124.36, 1555.0, 0.500),         // Pair(120, 120)
        127.03  to ShotParameters(127.03, 1495.0, 0.525),         // Pair(120, 108)
        131.12  to ShotParameters(131.12, 1630.0, 0.525),         // Pair(72, 36)
        135.44  to ShotParameters(135.44, 1630.0, 0.525),         // Pair(60, 24)
        135.79  to ShotParameters(135.79, 1675.0, 0.500),         // Pair(132, 120)
        141.00  to ShotParameters(141.00, 1630.0, 0.525),         // Pair(72, 24)
        141.68  to ShotParameters(141.68, 1650.0, 0.500),         // Pair(48, 12)
        146.01  to ShotParameters(146.01, 1650.0, 0.500),         // Pair(60, 12)
        147.32  to ShotParameters(147.32, 1630.0, 0.525),         // Pair(84, 24)
        149.59  to ShotParameters(149.59, 1650.0, 0.500),         // Pair(36, 0)
        151.18  to ShotParameters(151.18, 1650.0, 0.500),         // Pair(72, 12)
        152.76  to ShotParameters(152.76, 1650.0, 0.500),         // Pair(48, 0)
        156.79  to ShotParameters(156.79, 1650.0, 0.500),         // Pair(60, 0)
        157.10  to ShotParameters(157.10, 1650.0, 0.500),         // Pair(84, 12)
        161.62  to ShotParameters(161.62, 1650.0, 0.500),         // Pair(72, 0)
        163.68  to ShotParameters(163.68, 1650.0, 0.500),         // Pair(96, 12)
        167.17  to ShotParameters(167.17, 1650.0, 0.500),         // Pair(84, 0)
        173.37  to ShotParameters(173.37, 1650.0, 0.500),         // Pair(96, 0)
        180.16  to ShotParameters(180.16, 1650.0, 0.500)          // Pair(108, 0)
    )

    /**
     * Finds the shot parameters for the distance closest to the target.
     * If the distance is within a certain threshold it returns the entry.
     */

    fun getShot(distance: Double): ShotParameters? {
        var bestDistance = Double.MAX_VALUE
        var closestKey: Double? = null

        for (keyDistance in shooterLookupTable.keys) {
            val diff = kotlin.math.abs(distance - keyDistance)

            if (diff < bestDistance) {
                bestDistance = diff
                closestKey = keyDistance
            }
        }

        // Only return if the closest data point is within range
        return if (closestKey != null && bestDistance <= 6.0) {
            shooterLookupTable[closestKey]
        } else {
            null
        }
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
