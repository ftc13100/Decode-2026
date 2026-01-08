package org.firstinspires.ftc.teamcode.opModes.teleOp//package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Shooter Test & Tune")
class ShooterTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Shooter),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()
    val shooterCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                Shooter.spinning()
            }
            .setIsDone { false }
            .requires(Shooter)
    )

    override fun onInit() {
        shooterCommand()
        timer.reset()
        updateSignals()
    }

    override fun onUpdate() {
        updateSignals()
    }

    private fun updateSignals() {
        panelsTelemetry.addData("velocity", Shooter.shooter.velocity)
        panelsTelemetry.addData("target", Shooter.target)
        panelsTelemetry.update(telemetry)
    }
}