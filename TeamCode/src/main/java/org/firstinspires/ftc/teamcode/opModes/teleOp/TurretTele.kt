package org.firstinspires.ftc.teamcode.opModes.teleOp//package org.firstinspires.ftc.teamcode.opModes.teleOp

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
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.concurrent.timer
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.abs

@TeleOp(name = "TurretTele")
class TurretTele : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(NewTurret),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry

    private val timer = ElapsedTime()
    val turretCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                NewTurret.turretTrackCommand()
            }
            .requires(NewTurret)
    )

    override fun onInit() {
        turretCommand()
        timer.reset()
    }

    override fun onUpdate() {
        telemetry.addData("Target", NewTurret.turretState)
        telemetry.addData("Angle", NewTurret.turretAngle)
        panelsTelemetry.addData("Target", Turret.target)
        panelsTelemetry.addData("Position", Turret.turret.currentPosition)
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}


