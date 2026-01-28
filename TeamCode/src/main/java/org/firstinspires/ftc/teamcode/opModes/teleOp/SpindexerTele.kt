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
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.extensions.pedro.PedroComponent
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.GoalFinder
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "SpindexerTesting")
class SpindexerTele : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Intake, Spindexer, Gate
            ),
            BindingsComponent,
            BulkReadComponent,
        )
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()

    //tuning only
    val spindexCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                Spindexer.spin()
            }
            .requires(Spindexer)
    )

    override fun onInit() {
//        spindexCommand()
        timer.reset()
        updateSignals()
    }

    override fun onStartButtonPressed() {
        button { gamepad2.left_bumper }
            .whenBecomesTrue {
                Gate.gate_in()
                Intake.spinFast()
            }
            .whenBecomesFalse {
                Gate.gate_stop()
                Intake.spinStop()
            }

        button { gamepad2.x }
            .whenBecomesTrue (Spindexer.index0)

        button { gamepad2.y }
            .whenBecomesTrue (Spindexer.index1)

        button { gamepad2.b }
            .whenBecomesTrue (Spindexer.index2)

        button { gamepad2.a }
            .whenBecomesTrue(Spindexer.spinShot)
            .whenBecomesFalse(Spindexer.stopShot)
    }

    override fun onUpdate() {
//        Gate.gate_spindex()
        updateSignals()
    }

    private fun updateSignals() {
        //telemetry.addData("Dexer Position", Spindexer.spindexer.currentPosition)
//        panelsTelemetry.addData("Position", Spindexer.spindexer.currentPosition)
//        panelsTelemetry.addData("Target", Spindexer.target)

        telemetry.addData("Dexer Pos", Spindexer.spindexer.currentPosition)
        telemetry.addData("S0 ", Spindexer.detectColorRGB(Spindexer.color0))
        telemetry.addData("S1 ", Spindexer.detectColorRGB(Spindexer.color1))
        telemetry.addData("S2 ", Spindexer.detectColorRGB(Spindexer.color2))
        telemetry.update()
        panelsTelemetry.update(telemetry)

    }
}