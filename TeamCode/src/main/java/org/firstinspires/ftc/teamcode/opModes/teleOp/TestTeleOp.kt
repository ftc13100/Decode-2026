package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.DriveController

@TeleOp(name = "Sachet's Testing")
class TestTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            SubsystemComponent(DriveController),
            PedroComponent(Constants::createFollower),
        )
    }

    override fun onInit() {
        follower.pose = Pose(72.0, 72.0, Math.toRadians(90.0))
        Gamepads.gamepad1.a whenTrue DriveController::pointToTarget
    }

    override fun onUpdate() {
        telemetry.update()
    }
}