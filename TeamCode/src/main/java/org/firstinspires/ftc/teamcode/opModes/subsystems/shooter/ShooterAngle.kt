package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object ShooterAngle: Subsystem {

    private val servo = ServoEx("angle")

    val up = SetPosition(servo, 0.0).requires(this)
    val down = SetPosition(servo, 0.4).requires(this)


}