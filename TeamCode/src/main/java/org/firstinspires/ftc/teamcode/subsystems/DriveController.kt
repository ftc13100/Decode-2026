package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads

object DriveController: Subsystem {
    override val defaultCommand =
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
        )
            .apply { scalar = 0.9 }
            .requires(this)
}