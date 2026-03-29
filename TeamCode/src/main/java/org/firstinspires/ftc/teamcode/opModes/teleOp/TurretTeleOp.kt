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
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
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


    override fun onInit() {
        NewTurret.trackTarget()
    }

    override fun onUpdate() {
        // Show current turret position
        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("newX", NewTurret.newX)
        telemetry.addData("newY", NewTurret.newY)
        panelsTelemetry.addData("Angular Vel", follower.angularVelocity)
        panelsTelemetry.addData("target pos", NewTurret.targetServoPosition)


        // Update panel and telemetry
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}