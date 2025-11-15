package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret

@TeleOp(name = "Turret Test & Tune")
class TurretTeleOp : NextFTCOpMode() {
    init {
        addComponents(
        SubsystemComponent(
            Turret, blueLime
        ),
        BulkReadComponent,
        BindingsComponent
        )
    }
    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val timer = ElapsedTime()
    private lateinit var limelight: Limelight3A

    override fun onInit() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 11
        limelight.pipelineSwitch(1)
        limelight.start()
        timer.reset()
    }

    override fun onStartButtonPressed() {
        CommandManager.scheduleCommand(
            Turret.toMiddle.perpetually()
        )

//        button { gamepad1.right_bumper }
//            .whenTrue {
//                Turret.toRight()
//            }
//            .whenFalse {
//                Turret.spinZero()
//            }
//
//        button { gamepad1.left_bumper }
//            .whenTrue {
//                Turret.toLeft()
//            }
//            .whenFalse {
//                Turret.spinZero()
//            }

    }

    override fun onUpdate() {
        BindingManager.update()
        val result = limelight.latestResult
        if (result != null && result.isValid) {
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
        } else {
            telemetry.addData("Limelight", "Target not found")
            // turret.stop()
        }
        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
        updateSignals()
    }

    private fun updateSignals() {
        panelsTelemetry.addData("velocity", Turret.turret.velocity)
        panelsTelemetry.addData("target tx", limelight.latestResult.tx)
        panelsTelemetry.update(telemetry)
    }
}

