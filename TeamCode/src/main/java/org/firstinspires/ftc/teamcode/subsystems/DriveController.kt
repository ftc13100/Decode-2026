package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.extensions.pedro.TurnTo
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import kotlin.math.PI
import kotlin.math.atan2

object DriveController: Subsystem {
    override val defaultCommand =
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
        )
            .apply { scalar = 0.9 }
            .requires(this)

    fun pointToTarget() {
        val target = Pose(3.0, 144.0)
        val diff = target - follower.pose

        val angle =
            if (PoseStorage.blueAlliance)
                atan2(diff.x, diff.y)
            else
                PI + atan2(diff.x, diff.y)

        ActiveOpMode.telemetry.addData("Angle Turned", angle.rad.normalized)

        TurnTo(angle.rad.normalized)
            .requires(this)
            .schedule()
    }
}