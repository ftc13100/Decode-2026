package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Shooter : Subsystem {
    @JvmField var target = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.002, 0.0, 0.0)
    @JvmField var basicFFParameters = BasicFeedforwardParameters(0.000385, 0.0, 0.095)

    val shooter1 = MotorEx("shooter1").brakeMode()
    val shooter2 = MotorEx("shooter2").brakeMode()
    var shooterActive = false
    var shooterReady = false
    var shooterReadyMs: Double = 0.00
    private val runtime = ElapsedTime()

    val controller = controlSystem {
        velPid(velPIDCoefficients)
        basicFF(basicFFParameters)
    }

    override fun periodic() {
        if (shooterActive) {
            val motorPower = controller.calculate(
                shooter1.state
            )
            shooter1.power = motorPower
            shooter2.power = motorPower

        } else {
            shooter1.power = 0.0
        }
    }

    fun spinAtSpeed(speed: Double) =
        InstantCommand {
            target = speed
            shooterActive = true
            shooterReady = false
            shooterReadyMs = 0.00
            runtime.reset()
        }.then(
            RunToVelocity(controller, speed, 21.0),
            InstantCommand {
                shooterReady = true
                shooterReadyMs = runtime.milliseconds()
            }
        ).setInterruptible(true).requires(this)

    val stallShooter = spinAtSpeed(1000.0)

    val stopShooter =
        InstantCommand {
            shooterActive = false
            shooterReady = false
            target = 0.0
        }.requires(this)


    fun spinning() {
        shooterActive = true
        controller.goal = KineticState(velocity = target)
        val motorPower = controller.calculate(
            KineticState(velocity = shooter1.velocity)
        )
        shooter1.power = motorPower
        shooter2.power = motorPower
    }
}