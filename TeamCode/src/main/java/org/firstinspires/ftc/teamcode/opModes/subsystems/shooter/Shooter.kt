package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Shooter : Subsystem {
    lateinit var shooter: MotorEx
    @JvmField var target = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)

    override fun initialize() {
        shooter = MotorEx("shooter")   // moved here
        shooter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun spin() {
        shooter.power = 0.9
    }

    fun stop() {
        shooter.power = 0.0
    }

    fun spinning() {
        val controller = controlSystem {
            velPid(velPIDCoefficients)
        }

        controller.goal = KineticState(velocity = target)

        shooter.power = controller.calculate(
            KineticState(velocity = shooter.velocity)
        )
    }
}
