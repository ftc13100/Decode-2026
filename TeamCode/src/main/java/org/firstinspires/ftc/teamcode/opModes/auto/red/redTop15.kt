package org.firstinspires.ftc.teamcode.opModes.auto.red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.TurnTo
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.DeadhuzzLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GoToSecretTunnel
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GoToShot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.Leave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.TheGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.buildPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.eatup
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.pickUpPGP2
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.startPose
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redTop15")
class redTop15 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto,
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
                    Shooter.spinAtSpeed(1180.0),
                    FollowPath(GoToShot),
                    TurretAuto.toLeft,
                    Gate.gate_open,
                ),
                ParallelDeadlineGroup(
                    Delay(1.8.seconds),
                    Intake.spinFast
                ),
                ParallelGroup(
                    FollowPath(PGPfirst),
                    Gate.gate_close
                ),
                FollowPath(PGPsecond, holdEnd = true, maxPower = 1.0),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_open,
                ),
                ParallelDeadlineGroup(
                    Delay(1.8.seconds),
                    Intake.spinFast
                ),
                ParallelGroup(
                    FollowPath(TheGate),
                    Gate.gate_close,
                ),
                FollowPath(eatup),
                Delay(1.8),
                ParallelGroup(
                    Intake.spinStop,
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_open,
                ),
                ParallelDeadlineGroup(
                    Delay(1.8.seconds),
                    Intake.spinFast
                ),
                ParallelDeadlineGroup(
                    FollowPath(PPGsecond, holdEnd = true, maxPower = 1.0),
                    Gate.gate_close,
                    Intake.spinFast
                ),
                ParallelGroup(
                    Intake.spinStop,
                    FollowPath(PPGtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_open,
                ),
                ParallelDeadlineGroup(
                    Delay(1.8.seconds),
                    Intake.spinFast
                ),
                ParallelDeadlineGroup(
                    FollowPath(GPPfirst),
                    Gate.gate_close,
                    Intake.spinFast
                ),
                FollowPath(GPPsecond, holdEnd = true, maxPower = 1.0),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(Leave),
                    ShooterAngle.angle_kindaUP,
                    Gate.gate_open
                ),
                ParallelDeadlineGroup(
                    Delay(1.8.seconds),
                    Intake.spinFast
                ),
                ParallelGroup(
                    FollowPath(DeadhuzzLeave),
                    Shooter.stopShooter,
                    Gate.gate_close,
                    Intake.spinStop,
                )
            )

    override fun onInit() {
        follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(startPose)
        buildPaths()
        PoseStorage.blueAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = follower.pose
    }
}