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
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate

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

    override fun onStartButtonPressed() {
        button { gamepad2.a }
            .whenBecomesTrue {

            }
            .whenBecomesFalse {

            }
    }

    override fun onUpdate() {
//        Gate.gate_spindex()
        updateSignals()
    }

    private fun updateSignals() {
        //telemetry.addData("Dexer Position", Spindexer.spindexer.currentPosition)
//        panelsTelemetry.addData("Position", Spindexer.spindexer.currentPosition)
//        panelsTelemetry.addData("Target", Spindexer.target)
        // Sensor 0
//        val c0 = Spindexer.color0.normalizedColors
//        telemetry.addData("S0 Hue", "%.1f°", Spindexer.getHue(Spindexer.color0))
//        telemetry.addData("S0 RGB", "R:%.2f G:%.2f B:%.2f", c0.red, c0.green, c0.blue)
//
//        // Sensor 1
//        val c1 = Spindexer.color1.normalizedColors
//        telemetry.addData("S1 Hue", "%.1f°", Spindexer.getHue(Spindexer.color1))
//        telemetry.addData("S1 RGB", "R:%.2f G:%.2f B:%.2f", c1.red, c1.green, c1.blue)
//
//        // Sensor 2
//        val c2 = Spindexer.color2.normalizedColors
//        telemetry.addData("S2 Hue", "%.1f°", Spindexer.getHue(Spindexer.color2))
//        telemetry.addData("S2 RGB", "R:%.2f G:%.2f B:%.2f", c2.red, c2.green, c2.blue)
        telemetry.addData("S0 State", Spindexer.detectColorRGB(Spindexer.color0))
        telemetry.addData("S1 State", Spindexer.detectColorRGB(Spindexer.color1))
        telemetry.addData("S2 State", Spindexer.detectColorRGB(Spindexer.color2))
        telemetry.update()
        panelsTelemetry.update(telemetry)

    }
}