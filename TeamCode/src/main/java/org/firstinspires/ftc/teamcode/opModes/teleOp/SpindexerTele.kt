package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import kotlin.math.abs

@TeleOp(name = "SpindexerTesting")
class SpindexerTele : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Spindexer),
            BulkReadComponent,
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()

    val spindexCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                Spindexer.spin()
            }
            .requires(Spindexer)
    )

    override fun onInit() {
        spindexCommand()
        timer.reset()
        updateSignals()
    }

    override fun onUpdate() {
        updateSignals()
    }

    private fun updateSignals() {
        //telemetry.addData("Dexer Position", Spindexer.spindexer.currentPosition)
        panelsTelemetry.addData("Position", Spindexer.spindexer.currentPosition)
        panelsTelemetry.addData("Target", Spindexer.target)
        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}