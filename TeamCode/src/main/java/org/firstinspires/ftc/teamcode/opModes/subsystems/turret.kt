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
//import dev.nextftc.hardware.powerable.SetPower

@Configurable
object turret : Subsystem {
    val turret = MotorEx("turret").brakeMode()

    fun spinRight() {
        turret.power = 0.3
    }

    fun spinLeft() {
        turret.power = -0.3
    }

    fun stop () {
        turret.power= 0.0
    }
//    val toRight = SetPower(turret, 0.3)
//    val toStop = SetPower(turret, 0.0)
//    val toLeft = SetPower(turret, -0.3)
}
