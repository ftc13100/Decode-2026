package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower


@TeleOp(name = "PedroTele")
class RobotCentricTeleop : NextFTCOpMode() {
override fun onStartButtonPressed() {

    val driverControlled = PedroDriverControlled(
        Gamepads.gamepad1.leftStickY,
        Gamepads.gamepad1.leftStickX,
        Gamepads.gamepad1.rightStickX
    )
    driverControlled.schedule()
    PedroComponent.follower.startTeleopDrive()
    }
}