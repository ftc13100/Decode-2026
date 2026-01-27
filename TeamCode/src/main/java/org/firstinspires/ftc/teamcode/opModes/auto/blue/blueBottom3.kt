package org.firstinspires.ftc.teamcode.opModes.auto.blue

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "blueBottom3")
class blueBottom3 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto, blueAutoPaths
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Shooter.spinAtSpeed(1450.0),
                    TurretAuto.toLeft,
                    Gate.gate_in,
                    FollowPath(blueAutoPaths.bottomShoot)

                ),
                Intake.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    Intake.spinStop,
                    Gate.gate_stop
                ),
                ParallelGroup(
                    TurretAuto.toLeft,
                    FollowPath(blueAutoPaths.bottomLeave),
                    Gate.gate_stop
                )
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_stop()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(blueAutoPaths.bottomStartPose)
        blueAutoPaths.buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
    }
}