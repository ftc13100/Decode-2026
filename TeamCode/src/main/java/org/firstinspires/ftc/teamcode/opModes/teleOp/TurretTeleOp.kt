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
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.concurrent.timer
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.turret
import kotlin.math.abs

@TeleOp(name = "Turret Test & Tune")
class TurretTeleOp : NextFTCOpMode() {

    init {
        addComponents(
        SubsystemComponent(
             blueLime, Shooter, Gate, Intake
        ),
        BulkReadComponent,
        BindingsComponent
        )
    }

    private lateinit var limelight: Limelight3A
    private var previous_tx = 0.0

    override fun onInit() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 11
        limelight.pipelineSwitch(1)
        limelight.start()

    }

    override fun onStartButtonPressed() {
        button { gamepad1.a }
            .whenBecomesTrue {
                Shooter.full()
            }
        button { gamepad1.b }
            .whenBecomesTrue {
                Shooter.half()
            }
        button { gamepad1.x }
            .whenBecomesTrue {
                Shooter.quarter()
            }
        button { gamepad1.y }
            .whenBecomesTrue {
                Shooter.zero()
            }
        button { gamepad1.right_bumper }
            .whenBecomesTrue {
                intake.power = 0.7
            }
            .whenBecomesFalse {
                intake.power = 0.0
            }
        button { gamepad1.dpad_up }
            .whenBecomesTrue {
                Gate.gate_open()
            }
        button { gamepad1.dpad_down }
            .whenBecomesTrue {
                Gate.gate_close()
            }

    }


    override fun onUpdate() {
        telemetry.addData("Shooter velocity", Shooter.shooter.velocity)

        BindingManager.update()
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
            if (abs(result.tx - previous_tx) > 4.0) {
                CommandManager.scheduleCommand(
                    turret.moveToTag(result.tx)
                )
            }
            previous_tx = result.tx
        } else {
            telemetry.addData("Limelight", "Target not found")
            // turret.stop()
        }
        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }
}

