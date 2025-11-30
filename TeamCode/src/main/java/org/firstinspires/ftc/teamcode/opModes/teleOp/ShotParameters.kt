package org.firstinspires.ftc.teamcode.opModes.teleOp

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle

data class ShotParameters(val x: Double, val y: Double, val velocity: Double, val angle: Double)

class ShooterController {

    private val shooterLookupTable: Map<Pair<Double, Double>, ShotParameters> = mapOf(
        //first = x, second = y
        //listing at 12 inch intervals, lookup written first match within 6 inches
        //top part starts here
        Pair(24.0, 132.0) to ShotParameters(24.0, 132.0,950.0, 0.7),
        Pair(36.0, 132.0) to ShotParameters(36.0, 132.0,975.0, 0.7),
        Pair(48.0, 132.0) to ShotParameters(48.0, 132.0, 1000.0, 0.7),
        Pair(60.0, 132.0) to ShotParameters(60.0, 132.0, 1100.0, 0.7),
        Pair(72.0, 132.0) to ShotParameters(72.0, 132.0, 1200.0, 0.7),
        Pair(84.0, 132.0) to ShotParameters(84.0, 132.0, 1325.0, 0.7),
        Pair(96.0, 132.0) to ShotParameters(96.0, 132.0,1450.0, 0.7),
        Pair(108.0, 132.0) to ShotParameters(108.0, 132.0, 1525.0, 0.7),
        Pair(120.0, 132.0) to ShotParameters(120.0, 132.0, 1600.0, 0.7),
        Pair(12.0, 120.0) to ShotParameters(12.0, 120.0, 950.0, 0.7),
        Pair(24.0, 120.0) to ShotParameters(24.0, 120.0, 950.0, 0.7),
        Pair(36.0, 120.0) to ShotParameters(36.0, 120.0, 975.0, 0.65),
        Pair(48.0, 120.0) to ShotParameters(48.0, 120.0, 1000.0, 0.6),
        Pair(60.0, 120.0) to ShotParameters(60.0, 120.0, 1100.0, 0.6),
        Pair(72.0, 120.0) to ShotParameters(72.0, 120.0, 1200.0, 0.6),
        Pair(84.0, 120.0) to ShotParameters(84.0, 120.0, 1325.0, 0.55),
        Pair(96.0, 120.0) to ShotParameters(96.0, 120.0,1450.0, 0.5),
        Pair(108.0, 120.0) to ShotParameters(108.0, 120.0,1525.0, 0.5),
        Pair(120.0, 120.0) to ShotParameters(120.0, 120.0,1600.0, 0.5),
        Pair(132.0, 120.0) to ShotParameters(132.0, 120.0,1600.0, 0.5),
        Pair(24.0, 108.0) to ShotParameters(24.0, 108.0,1025.0, 0.65),
        Pair(36.0, 108.0) to ShotParameters(36.0, 108.0,1025.0, 0.65),
        Pair(48.0, 108.0) to ShotParameters(48.0, 108.0,1050.0, 0.6),
        Pair(60.0, 108.0) to ShotParameters(60.0, 108.0,1125.0, 0.6),
        Pair(72.0, 108.0) to ShotParameters(72.0, 108.0,1200.0, 0.6),
        Pair(84.0, 108.0) to ShotParameters(84.0, 108.0,1267.0, 0.575),
        Pair(96.0, 108.0) to ShotParameters(96.0, 108.0,1375.0, 0.55),
        Pair(108.0, 108.0) to ShotParameters(108.0, 108.0,1450.0, 0.55),
        Pair(120.0, 108.0) to ShotParameters(120.0, 108.0,1450.0, 0.55),
        Pair(36.0, 96.0) to ShotParameters(36.0, 96.0,1100.0, 0.6),
        Pair(48.0, 96.0) to ShotParameters(48.0, 96.0,1100.0, 0.6),
        Pair(60.0, 96.0) to ShotParameters(60.0, 96.0,1150.0, 0.6),
        Pair(72.0, 96.0) to ShotParameters(72.0, 96.0,1200.0, 0.6),
        Pair(84.0, 96.0) to ShotParameters(84.0, 96.0,1250.0, 0.6),
        Pair(96.0, 96.0) to ShotParameters(96.0, 96.0,1300.0, 0.6),
        Pair(108.0, 96.0) to ShotParameters(108.0, 96.0,1300.0, 0.6),
        Pair(48.0, 84.0) to ShotParameters(48.0, 84.0,1250.0, 0.575),
        Pair(60.0, 84.0) to ShotParameters(60.0, 84.0,1250.0, 0.575),
        Pair(72.0, 84.0) to ShotParameters(72.0, 84.0,1275.0, 0.55),
        Pair(84.0, 84.0) to ShotParameters(84.0, 84.0,1325.0, 0.55),
        Pair(96.0, 84.0) to ShotParameters(96.0, 84.0,1325.0, 0.55),
        Pair(72.0, 72.0) to ShotParameters(72.0, 72.0,1350.0, 0.5),
        Pair(72.0, 60.0) to ShotParameters(72.0, 60.0,1350.0, 0.5),
        //bottom part starts here
        Pair(72.0, 36.0) to ShotParameters(72.0, 36.0, 1700.0, 0.5),
        Pair(60.0, 24.0) to ShotParameters(60.0, 24.0, 1700.0, 0.5),
        Pair(72.0, 24.0) to ShotParameters(72.0, 24.0, 1700.0, 0.5),
        Pair(84.0, 24.0) to ShotParameters(84.0, 24.0, 1700.0, 0.5),
        Pair(48.0, 12.0) to ShotParameters(48.0, 12.0, 1700.0, 0.5),
        Pair(60.0, 12.0) to ShotParameters(60.0, 12.0, 1700.0, 0.5),
        Pair(72.0, 12.0) to ShotParameters(72.0, 12.0, 1700.0, 0.5),
        Pair(84.0, 12.0) to ShotParameters(84.0, 12.0, 1700.0, 0.5),
        Pair(96.0, 12.0) to ShotParameters(96.0, 12.0, 1700.0, 0.5),
        Pair(36.0, 0.0) to ShotParameters(36.0, 0.0,1700.0, 0.5),
        Pair(48.0, 0.0) to ShotParameters(48.0, 0.0,1700.0, 0.5),
        Pair(60.0, 0.0) to ShotParameters(60.0, 0.0,1700.0, 0.5),
        Pair(72.0, 0.0) to ShotParameters(72.0, 0.0,1700.0, 0.5),
        Pair(84.0, 0.0) to ShotParameters(84.0, 0.0,1700.0, 0.5),
        Pair(96.0, 0.0) to ShotParameters(96.0, 0.0,1700.0, 0.5),
        Pair(108.0, 0.0) to ShotParameters(108.0, 0.0,1700.0, 0.5),
        )
    fun getShot(x: Double, y: Double): ShotParameters? {
        var bestKey: Pair<Double, Double>? = null
        for ((keyX, keyY) in shooterLookupTable.keys) {
            val dx = kotlin.math.abs(x - keyX)
            val dy = kotlin.math.abs(y - keyY)

            if (dx <= 6.0 && dy <= 6.0) {
                bestKey = Pair(keyX, keyY)
                break
            }
        }

        return if (bestKey != null) {
            shooterLookupTable[bestKey]
        } else {
            null
        }
    }

    fun applyShot(params: ShotParameters) {
        ShooterAngle.targetPosition = params.angle

        CommandManager.scheduleCommand(
            ShooterAngle.update()
        )

        CommandManager.scheduleCommand(
        Shooter.spinAtSpeed(params.velocity)
        )
    }
}
