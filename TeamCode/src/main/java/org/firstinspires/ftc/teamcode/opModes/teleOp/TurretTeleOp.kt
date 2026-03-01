package org.firstinspires.ftc.teamcode.opModes.teleOp//package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
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

    override fun onInit() {
        PoseStorage.blueAlliance = false
        follower.pose = Pose(72.0, 72.0, Math.toRadians(90.0))
        Turret.trackTarget()
    }

    override fun onUpdate() {
        telemetry.addData("Turret Position", Turret.turret.currentPosition)
//        panelsTelemetry.addData("Target", Turret.target)
//        panelsTelemetry.addData("Position", Turret.turret.currentPosition)
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}


