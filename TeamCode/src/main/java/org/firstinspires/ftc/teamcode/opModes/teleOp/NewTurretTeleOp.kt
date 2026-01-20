package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret

@TeleOp
class NewTurretTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(NewTurret),
            BindingsComponent,
            BulkReadComponent
        )
    }

    override fun onInit() {
        Gamepads.gamepad1.a whenBecomesTrue NewTurret.increment
        Gamepads.gamepad1.b whenBecomesTrue NewTurret.decrement
    }
}