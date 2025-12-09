package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackElement
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforward
import dev.nextftc.control.feedforward.FeedforwardElement
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Turret : Subsystem {
    @JvmField var target = 0.0
    @JvmField var turretActive: Boolean = false
    @JvmField var turretReady: Boolean = false
    @JvmField var turretReadyMs: Double = 0.00
    @JvmField var startPosition: Double = 0.0
    @JvmField var leftLimit: Double = 0.0
    @JvmField var rightLimit: Double = 0.0
    @JvmField var posPIDCoefficients = PIDCoefficients(0.0095, 0.000000, 0.0001)

    val turret = MotorEx("turret").brakeMode()

    private val runtime = ElapsedTime()

    fun setStartPos()  {
        turret.zero()
        startPosition = turret.currentPosition
        target = startPosition
        rightLimit = startPosition + 900.0
        leftLimit = startPosition - 900.0
    }

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

    fun spinToPos(pos: Double) {
        target = pos.coerceIn(leftLimit, rightLimit)
        CommandManager.scheduleCommand(
            InstantCommand {
                turretActive = true
                turretReady = false
                runtime.reset()
                turretReadyMs = 0.0
            }.then(
                RunToPosition(controlSystem, target, 3.0),
                InstantCommand {
                    turretReady = true
                    turretReadyMs = runtime.milliseconds()
                }
            ).setInterruptible(true).requires(this)
        )
    }


    override fun periodic() {
            if (turretActive) {
                turret.power = controlSystem.calculate(turret.state)
            } else {
                turret.power = 0.0
            }
        }
    }
