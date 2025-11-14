package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import com.bylazar.telemetry.PanelsTelemetry
import kotlin.concurrent.timer
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.turret

@TeleOp(name = "Turret Test & Tune")
class TurretTeleOp : NextFTCOpMode() {
    init {
        SubsystemComponent(
            turret
        )
        BulkReadComponent
        BindingsComponent
    }
    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()
    val shooterCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
turret.stop()         }
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
        panelsTelemetry.addData("mohitPatil", turret.turret.currentPosition)
        panelsTelemetry.addData("target", turret.target)
        panelsTelemetry.update(telemetry)
    }
}