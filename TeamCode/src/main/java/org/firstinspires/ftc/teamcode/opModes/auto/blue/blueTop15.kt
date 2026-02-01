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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPtoShot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPtoShotMove
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
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret.trackTarget
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
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage, Turret
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val startPose = Pose(123.500, 122.200, Math.toRadians(36.0))


val turn = follower.pathBuilder().addPath(
    BezierLine(
        Pose(20.500, 122.200),
        Pose(20.500, 122.200)
    )
).setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(136.0))
    .setReversed()
    .build()

val shoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(20.500, 122.200),
        Pose(60.000, 84.000)
    )
).setLinearHeadingInterpolation(Math.toRadians(140.0), Math.toRadians(-180.0))
    .setReversed()
    .build()

val shootPGP = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(60.000, 84.000),
        Pose(52.683, 48.669),
        Pose(36.991, 65.990),
        Pose(73.646, 58.941),
        Pose(15.000, 60.000),
        Pose(11.000, 60.000)
    )
).setTangentHeadingInterpolation()
    .build()

val PGPshoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(11.000, 60.000),
        Pose(60.000, 84.000)
    )
).setTangentHeadingInterpolation()
    .setReversed()
    .build()

val shootGate = follower.pathBuilder().addPath(
    BezierLine(
        Pose(60.000, 84.000),
        Pose(16.100, 65.100)
    )
).setLinearHeadingInterpolation(Math.toRadians(-154.0), Math.toRadians(-180.0))
    .build()

val gateIntake = follower.pathBuilder().addPath(
    BezierLine(
        Pose(16.100, 65.100),
        Pose(8.000, 48.890)
    )
).setLinearHeadingInterpolation(Math.toRadians(-180.0), Math.toRadians(90.0))
    .build()

val intakeShoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(8.000, 48.890),
        Pose(60.000, 84.000)
    )
).setTangentHeadingInterpolation()
    .build()

val shootPPG = follower.pathBuilder().addPath(
    BezierLine(
        Pose(60.000, 84.000),
        Pose(16.000, 84.000)
    )
).setTangentHeadingInterpolation()
    .build()

val PPGshoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(16.000, 84.000),
        Pose(60.000, 84.000)
    )
).setTangentHeadingInterpolation()
    .setReversed()
    .build()

val shootGPP = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(60.000, 84.000),
        Pose(42.899, 51.178),
        Pose(47.796, 28.150),
        Pose(46.044, 37.937),
        Pose(11.000, 36.000)
    )
).setTangentHeadingInterpolation()
    .build()

val GPPshoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(11.000, 36.000),
        Pose(60.000, 84.000)
    )
).setTangentHeadingInterpolation()
    .setReversed()
    .build()

val shootLeave = follower.pathBuilder().addPath(
    BezierLine(
        Pose(60.000, 84.000),
        Pose(22.340, 72.500)
    )
).setTangentHeadingInterpolation()
    .build()
    val autoRoutine: Command
        get() =
            SequentialGroup(
                FollowPath(turn),
                ParallelGroup(
                    FollowPath(shoot),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1200.0),
                    Gate.gate_open
                ),
                Intake.spinFastAuto,
                Delay(2.3),
                ParallelGroup(
                    FollowPath(shootPGP),
                    Gate.gate_close
                ),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PGPshoot),
                    Gate.gate_open
                ),
                Intake.spinFastAuto,
                Delay(2.3),
                ParallelGroup(
                    FollowPath(shootGate),
                    Gate.gate_close
                ),
                FollowPath(gateIntake),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(intakeShoot),
                    Gate.gate_open
                ),
                Intake.spinFastAuto,
                Delay(2.3),
                ParallelGroup(
                    FollowPath(shootPPG),
                    Gate.gate_close
                ),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PPGshoot),
                    Gate.gate_open
                ),
                Intake.spinFastAuto,
                Delay(2.3),
                ParallelGroup(
                    FollowPath(shootGPP),
                    Gate.gate_close
                ),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(GPPshoot),
                    Gate.gate_open
                ),
                Intake.spinFastAuto,
                Delay(2.3),
                ParallelGroup(
                    FollowPath(shootLeave),
                    Gate.gate_close,
                    Shooter.stopShooter,
                ),
            )

    override fun onInit() {
        follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(startPose)
        buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        trackTarget()
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = follower.pose
    }
}