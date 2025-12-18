package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
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
}