package org.firstinspires.ftc.teamcode.opModes.teleOp

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle

data class ShotParameters(val velocity: Double, val angle: Double)

class ShooterController {

    private val shooterLookupTable: Map<Pair<Double, Double>, ShotParameters> = mapOf(
//        Pair(24.0, 120.0) to ShotParameters(-950.0, 0.2),
//        Pair(48.0, 96.0) to ShotParameters(-1100.0, 0.1),
//        Pair(72.0, 72.0) to ShotParameters(-1350.0, 0.0),
//        Pair(48.0, 120.0) to ShotParameters(-1000.0, 0.1),
//        Pair(72.0, 120.0) to ShotParameters(-1200.0, 0.1),
//        Pair(72.0, 96.0) to ShotParameters(-1200.0, 0.1),
//        Pair(72.0, 24.0) to ShotParameters(-1700.0, 0.0),
//        Pair(96.0, 96.0) to ShotParameters(-1300.0, 0.1),
//        Pair(120.0, 120.0) to ShotParameters(-1600.0, 0.0),
//        Pair(96.0, 120.0) to ShotParameters(-1450.0, 0.0),
//        Pair(88.5, 8.25) to ShotParameters(-1650.0, 0.0),
//test pair 0.0
        Pair(0.0,0.0) to ShotParameters(-1600.0,0.6),
        Pair(120.0, 24.0) to ShotParameters(-950.0, 0.7),
        Pair(96.0, 48.0) to ShotParameters(-1100.0, 0.6),
        Pair(72.0, 72.0) to ShotParameters(-1350.0, 0.5),
        Pair(120.0, 48.0) to ShotParameters(-1000.0, 0.6),
        Pair(120.0, 72.0) to ShotParameters(-1200.0, 0.6),
        Pair(96.0, 72.0) to ShotParameters(-1200.0, 0.6),
        Pair(24.0, 72.0) to ShotParameters(-1700.0, 0.5),
        Pair(96.0, 96.0) to ShotParameters(-1300.0, 0.6),
        Pair(120.0, 120.0) to ShotParameters(-1600.0, 0.5),
        Pair(120.0, 96.0) to ShotParameters(-1450.0, 0.5),
        Pair(8.25, 88.5) to ShotParameters(-1650.0, 0.5)

    )

    fun getShot(x: Double, y: Double): ShotParameters? {
        var bestKey: Pair<Double, Double>? = null

        for ((keyX, keyY) in shooterLookupTable.keys) {
            val dx = kotlin.math.abs(x - keyX)
            val dy = kotlin.math.abs(y - keyY)

            if (dx <= 5.0 && dy <= 5.0) {
                bestKey = Pair(keyX, keyY)
                break
            }
        }

        return if (bestKey != null) {
            telemetry.addData("Found shot near", "${bestKey.first}, ${bestKey.second}")
            shooterLookupTable[bestKey]
        } else {
            telemetry.addData("No shot found", "")
            null
        }
    }

    fun applyShot(params: ShotParameters) {
        ShooterAngle.targetPosition = params.angle

        CommandManager.scheduleCommand(
        Shooter.spinAtSpeed(params.velocity)
        )

        CommandManager.scheduleCommand(
            ShooterAngle.update()
        )

        telemetry.log().add("Applying shot: V=${params.velocity}, A=${params.angle}")
    }
}
