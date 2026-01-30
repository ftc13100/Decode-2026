package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Turret Test & Tune")
class TurretTeleOp : NextFTCOpMode() {

    init {
        addComponents(
        SubsystemComponent(Turret),
        BulkReadComponent,
        PedroComponent(Constants::createFollower)
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()

    val turretCommand =
        LambdaCommand()
            .setUpdate {
                Turret.turn(Turret.target)
            }
            .setIsDone { false }
            .requires(Turret)

    override fun onInit() {
        turretCommand()
        timer.reset()
    }

    override fun onUpdate() {
        telemetry.addData("Turret Position", Turret.turret.currentPosition)
        panelsTelemetry.addData("Target", Turret.target)
        panelsTelemetry.addData("Position", Turret.turret.currentPosition)
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}


