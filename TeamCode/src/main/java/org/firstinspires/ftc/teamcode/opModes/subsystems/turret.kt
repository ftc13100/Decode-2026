package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.shooter

//import dev.nextftc.hardware.powerable.SetPower


@Configurable
object turret : Subsystem {
    @JvmField var target = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.03, 0.01, 5.0,)
    val turret = MotorEx("turret").brakeMode()


    fun stop () {
        val controller = controlSystem {
            posPid(velPIDCoefficients)
        }
        controller.goal = KineticState(position = target)

        turret.power = controller.calculate(
            KineticState(position = turret.currentPosition))
    }

    fun spinRight() {
        turret.power = 0.7
    }

    fun spinLeft() {

        turret.power = -0.7
    }
}
