package org.firstinspires.ftc.teamcode.opModes.teleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Shooter
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

@TeleOp(name = "Shooter Test & Tune")
class ShooterTeleOp : NextFTCOpMode() {
    init {
        SubsystemComponent(
            Shooter
        )
        BulkReadComponent
        BindingsComponent
    }

    val shooterCommand = PerpetualCommand(
        LambdaCommand()
            .setUpdate {
                Shooter.spinning()
            }
            .requires(Shooter)
    )

    override fun onInit() {
        shooterCommand()
    }
}