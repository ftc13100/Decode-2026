package org.firstinspires.ftc.teamcode.opModes.teleOp

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Turret Test & Tune")
class TurretTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(NewTurret),
            BindingsComponent,
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()

    private val turretCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                // Nothing needed here; turret periodic handles updates
            }
            .requires(NewTurret)
    )

    override fun onInit() {
        turretCommand()
        timer.reset()
    }

    override fun onUpdate() {
        // Show current turret position
        telemetry.addData("Turret Position", NewTurret.targetServoPosition)

        // Add panel inputs for manual X/Y control
//        panelsTelemetry.addData("Manual X", NewTurret.manualX ?: 0.0)
//        panelsTelemetry.addData("Manual Y", NewTurret.manualY ?: 0.0)
        panelsTelemetry.addData("target pos", NewTurret.targetServoPosition)


        // Update panel and telemetry
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}