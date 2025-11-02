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

@TeleOp(name = "Shooter Test & Tune")
class ShooterTeleOp : NextFTCOpMode() {
    init {
        SubsystemComponent(
            Shooter
        )
        BulkReadComponent
        BindingsComponent
    }
    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()
    private var velocity = Shooter.shooter.velocity
    private var target = Shooter.target
    val shooterCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                Shooter.spinning()
            }
            .requires(Shooter)
    )
    override fun onInit() {
        shooterCommand()
        timer.reset()
    }

    override fun onUpdate() {
        panelsTelemetry.addData("velocity", velocity)
        panelsTelemetry.addData("target", target)
        panelsTelemetry.update(telemetry)
    }
}