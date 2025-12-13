package org.firstinspires.ftc.teamcode.opModes.teleOp//package org.firstinspires.ftc.teamcode.opModes.teleOp
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import dev.nextftc.core.commands.Command
//import dev.nextftc.core.commands.utility.LambdaCommand
//import dev.nextftc.core.commands.utility.PerpetualCommand
//import dev.nextftc.core.components.BindingsComponent
//import dev.nextftc.core.components.SubsystemComponent
//import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
//import dev.nextftc.ftc.NextFTCOpMode
//import dev.nextftc.ftc.components.BulkReadComponent
//import dev.nextftc.hardware.impl.MotorEx
//import com.bylazar.telemetry.PanelsTelemetry
//import kotlin.concurrent.timer
//import com.qualcomm.robotcore.util.ElapsedTime
//import dev.nextftc.extensions.pedro.PedroComponent
//import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower
//
//@TeleOp(name = "Shooter Test & Tune")
//class ShooterTeleOp : NextFTCOpMode() {
//    init {
//        addComponents(
//            SubsystemComponent(Shooter),
//            BulkReadComponent,
//            PedroComponent(Constants::createFollower)
//        )
//    }
//    private val panelsTelemetry = PanelsTelemetry.telemetry
//    private val timer = ElapsedTime()
//    val shooterCommand = PerpetualCommand(
//        LambdaCommand()
//            .setUpdate {
//                Shooter.spinning()
//            }
//            .requires(Shooter)
//    )
//    override fun onInit() {
//        shooterCommand()
//        timer.reset()
//        updateSignals()
//    }
//
//    override fun onUpdate() {
//        updateSignals()
//    }
//
//    private fun updateSignals() {
//        panelsTelemetry.addData("velocity", Shooter.shooter.velocity)
//        panelsTelemetry.addData("target", Shooter.target)
//        panelsTelemetry.update(telemetry)
//    }
//}