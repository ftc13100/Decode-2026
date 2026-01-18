package org.firstinspires.ftc.teamcode.opModes.auto.red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GoToShot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.Leave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.MohitHitGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.buildPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.startPose
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.IntakeAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redTop12")
class redTop12: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, IntakeAuto, Gate, PoseStorage,
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
                    FollowPath(GoToShot),
                    TurretAuto.toLeft,
                    Gate.gate_open
                ),
                IntakeAuto.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    IntakeAuto.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(PPGfirst),
                    Gate.gate_close
                ),
                IntakeAuto.spinFast,
                FollowPath(PPGsecond, holdEnd = true, maxPower = 0.65),
                IntakeAuto.spinStop,
                FollowPath(MohitHitGate),
                Delay(1.0.seconds),
                ParallelGroup(
                    FollowPath(PPGtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                IntakeAuto.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    IntakeAuto.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(PGPfirst),
                    Gate.gate_close,
                    IntakeAuto.spinFast
                ),
                FollowPath(PGPsecond, holdEnd = true, maxPower = 0.65),
                IntakeAuto.spinStop,
                ParallelGroup(
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                IntakeAuto.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Gate.gate_close,
                    FollowPath(GPPfirst),
                    IntakeAuto.spinFast
                ),
                FollowPath(GPPsecond, holdEnd = true, maxPower = 0.65),
                IntakeAuto.spinStop,
                ParallelGroup(
                    FollowPath(Leave),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                IntakeAuto.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    TurretAuto.toMid,
                    Gate.gate_close,
                    IntakeAuto.spinStop
                ),
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(startPose)
        buildPaths()
        PoseStorage.blueAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
    }
}