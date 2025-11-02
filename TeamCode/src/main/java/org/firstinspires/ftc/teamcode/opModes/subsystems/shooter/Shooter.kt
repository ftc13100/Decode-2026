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
object Shooter: Subsystem {
    val shooter = MotorEx("shooter")
    @JvmField
    var target = 0.0

    @JvmField
    var velPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)


//    val controller = controlSystem {
//        velPid(velPIDCoefficients)
//    }

    override fun initialize() {
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

        controller.goal = KineticState(
            velocity = target
        )

        shooter.power = controller.calculate(
            KineticState(
                velocity = shooter.velocity
            )
        )



        ActiveOpMode.telemetry.addData("Measured Velocity", shooter.velocity)
        ActiveOpMode.telemetry.addData("Target Velocity", target)

        ActiveOpMode.telemetry.update()
    }

}