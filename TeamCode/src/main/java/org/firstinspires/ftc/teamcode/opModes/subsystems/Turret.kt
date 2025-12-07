package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Turret : Subsystem {
    @JvmField
    var target = 0.0
    var turretActive: Boolean = false

    var startPosition: Double = 0.0
    var leftLimit: Double = 0.0
    var rightLimit: Double = 0.0


    @JvmField
    var posPIDCoefficients = PIDCoefficients(0.025, 0.0, 0.0)

    val turret = MotorEx("turret").brakeMode()


    val resetPos = InstantCommand {
     turret.zero()
    }.requires(this)

    val setStartPos = InstantCommand {
        startPosition = turret.currentPosition
        rightLimit = startPosition + 1500
        leftLimit = startPosition - 1500
    }.requires(this)


    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

//    fun spinToPos(pos: Double) =
//        InstantCommand{
//            turretActive = true
//            target = pos
//            RunToPosition(controlSystem, target)
//        }.setInterruptible(true).requires(this)

    fun spinToPos(pos: Double) =
        InstantCommand {
            if (pos > rightLimit) {
                target = rightLimit
            } else if (pos < leftLimit) {
                target = leftLimit
            } else {
                target = pos
            }
            turretActive = true
        }.then(
            RunToPosition(controlSystem, target, 0.0)
        ).setInterruptible(true).requires(this)

//    val toRight = RunToPosition(controlSystem, 300.0).requires(this)
//    val toMiddle = RunToPosition(controlSystem, 0.0).requires(this)
//    val toLeft = RunToPosition(controlSystem, -300.0).requires(this)

//    fun moveToTag (tagPos: Double) =
//        if (tagPos < 300 && tagPos > -300) {
//            RunToPosition(controlSystem, tagPos).requires(this)
//        } else {
//            InstantCommand {
//                spinZero()
//            }
//        }

    override fun periodic() {
        if (turretActive) {
            turret.power = controlSystem.calculate(turret.state)
        } else {
            turret.power = 0.0
        }
        }
    }
