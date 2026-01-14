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

@Autonomous(name = "blueTop12")
class blueTop12: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    FollowPath(blueAutoPaths.GoToShot),
                    TurretAuto.toRight,
                    Gate.gate_open
                ),
                Intake.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(blueAutoPaths.PPGfirst),
                    Gate.gate_close
                ),
                Intake.spinFast,
                FollowPath(blueAutoPaths.PPGsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                FollowPath(blueAutoPaths.MohitHitGate),
                Delay(1.0.seconds),
                ParallelGroup(
                    FollowPath(blueAutoPaths.PPGtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(blueAutoPaths.PGPfirst),
                    Gate.gate_close,
                    Intake.spinFast
                ),
                FollowPath(blueAutoPaths.PGPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(blueAutoPaths.PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Gate.gate_close,
                    FollowPath(blueAutoPaths.GPPfirst),
                    Intake.spinFast
                ),
                FollowPath(blueAutoPaths.GPPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(blueAutoPaths.Leave),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    TurretAuto.toMid,
                    Gate.gate_close,
                    Intake.spinStop
                ),
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(blueAutoPaths.startPose)
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