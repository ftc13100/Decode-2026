package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object ShooterAngle: Subsystem {

    private val servo = ServoEx("angle")

    var targetPosition: Double = 0.0

    fun update() {
        // Set the servo position directly
        servo.position = targetPosition
    }


    val angle_up = SetPosition(servo, 0.0).requires(this)

    val angle_middle = SetPosition(servo, 0.1).requires(this)

    val angle_down = SetPosition(servo, 0.2).requires(this)
}
