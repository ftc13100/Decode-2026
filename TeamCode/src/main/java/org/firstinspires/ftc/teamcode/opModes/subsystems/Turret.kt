package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackElement
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforward
import dev.nextftc.control.feedforward.FeedforwardElement
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Turret : Subsystem {
    @JvmField var target = 0.0
    @JvmField var turretActive: Boolean = false
    @JvmField var startPosition: Double = 0.0
    @JvmField var leftLimit: Double = 0.0
    @JvmField var rightLimit: Double = 0.0
    @JvmField var posPIDCoefficients = PIDCoefficients(0.015, 0.0, 0.0)
   // @JvmField var feedforward = FeedbackElement(0.0)
    val turret = MotorEx("turret").brakeMode()


    val resetPos = InstantCommand {
     turret.zero()
    }.requires(this)

    val setStartPos = InstantCommand {
        startPosition = turret.currentPosition
        rightLimit = startPosition + 1000
        leftLimit = startPosition - 1000
    }.requires(this)

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    fun turretPID() {
        turretActive = true
        controlSystem.goal = KineticState(position = target)

        turret.power = controlSystem.calculate(
            KineticState(position = turret.currentPosition)
        )
    }
//    fun spinToPos(pos: Double) =
//        InstantCommand{
//            turretActive = true
//            target = pos
//            RunToPosition(controlSystem, target)
//        }.setInterruptible(true).requires(this)

    fun spinToPos(pos: Double) =
        InstantCommand {
            if (pos < rightLimit) {
                target = rightLimit
            } else if (pos > leftLimit) {
                target = leftLimit
            } else {
                target = pos
            }
            turretActive = true
        }.then(
            RunToPosition(controlSystem, target, 0.0)
        ).setInterruptible(true).requires(this)


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
