package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.conditionals.switchCommand
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret.turretState
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret.turretTrackCommand
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer.spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret.heading
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goal
import kotlin.math.abs
import kotlin.math.atan2

object NewTurret : Subsystem {
    enum class TurretState {
        HOLD_POSITION,
        AUTO_TRACK,
        DISABLED
    }

    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    const val MAX_LIMIT_DEG = 210.0

    var turretPosition = 0.0
        set(value) {
            val safeVal = value.coerceIn(0.0, 0.7)

            if (::turret1.isInitialized and ::turret2.isInitialized) {
                turret1.position = safeVal
                turret2.position = safeVal
            }

            field = safeVal
        }

    var turretState = TurretState.AUTO_TRACK

    val turretAzimuth: Double
        get() = turretPosition * MAX_LIMIT_DEG


    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap["turret1"] as Servo
        turret2 = ActiveOpMode.hardwareMap["turret2"] as Servo
        turretPosition = 0.0
    }

    val turretAngle: Double
        get() = 300.0 * turretPosition


    val increment = InstantCommand {
        turretPosition += 0.001
    }

    val decrement = InstantCommand {
        turretPosition -= 0.001
    }

    val turretTrackCommand =
        InstantCommand {
                val x = abs(follower.pose.x)
                val y = abs(follower.pose.y)

                val targetAngle = if (PoseStorage.blueAlliance) {
                    Math.PI - atan2(abs(goal.y - y), abs(goal.x - x))
                } else {
                    atan2(abs(goal.y - y), abs(goal.x - (144.0 - x)))
                }

                turretPosition = targetAngle.rad.wrapped.inDeg
            }

    val disabledCommand = PerpetualCommand(
        InstantCommand { turretPosition = 0.0 }
    )

//    override val defaultCommand =
//        switchCommand(::turretState) {
//            case(TurretState.AUTO_TRACK, turretTrackCommand)
//            case(TurretState.HOLD_POSITION, NullCommand())
//            // Case above is a little stupid but its so that the other commands aren't scheduled
//            // (servo mode makes it so that like I don't need to do anything)
//            case(TurretState.DISABLED, disabledCommand)
//        }
//            .requires(this)
}