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
    @JvmField var target = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.0375, 0.15, 0.2)

    val shooter = MotorEx("shooter").brakeMode()



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
