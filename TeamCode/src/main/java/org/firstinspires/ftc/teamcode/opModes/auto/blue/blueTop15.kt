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

@Autonomous(name = "blueTop15")
class blueTop15 : NextFTCOpMode() {
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
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    FollowPath(blueAutoPaths.GoToShot),
                    TurretAuto.toLeft,
                    Gate.gate_in
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    FollowPath(blueAutoPaths.PGPfirst),
                    Gate.gate_stop
                ),
                FollowPath(blueAutoPaths.PGPsecond, holdEnd = true, maxPower = 0.8),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(blueAutoPaths.PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_in,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    FollowPath(blueAutoPaths.TheGate),
                    Gate.gate_stop
                ),
                Delay(2.2.seconds),
                ParallelGroup(
                    Intake.spinStop,
                    FollowPath(blueAutoPaths.PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_in,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    FollowPath(blueAutoPaths.PPGsecond, holdEnd = true, maxPower = 0.8),
                    Gate.gate_stop,
                    Intake.spinFast
                ),
                ParallelGroup(
                    Intake.spinStop,
                    FollowPath(blueAutoPaths.PPGtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_in,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),

                ParallelGroup(
                    FollowPath(blueAutoPaths.GPPfirst),
                    Gate.gate_stop,
                    Intake.spinFast
                ),
                FollowPath(blueAutoPaths.GPPsecond, holdEnd = true, maxPower = 0.8),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(blueAutoPaths.Leave),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_in,
                    TurretAuto.toMid
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    Gate.gate_stop,
                    Intake.spinStop,
                )
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_stop()
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
}