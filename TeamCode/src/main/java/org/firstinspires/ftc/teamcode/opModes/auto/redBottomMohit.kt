package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redBottomMohit")
class redBottomMohit: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val startPose = Pose(56.0, 7.5, Math.toRadians(90.0)).mirror()
    private val shootPose = Pose(56.0, 10.5, Math.toRadians(90.0)).mirror()

    private val leavePoint = Pose(36.49261083743842, 8.20935960591133, Math.toRadians(90.0)).mirror()
    private lateinit var Leave: PathChain
    private lateinit var shoot: PathChain

    private fun buildPaths() {
        shoot = follower.pathBuilder()
            .addPath(BezierLine(startPose,shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        Leave = follower.pathBuilder()
            .addPath(BezierLine(startPose,leavePoint))
            .setLinearHeadingInterpolation(shootPose.heading, leavePoint.heading)
            .build()
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Shooter.spinAtSpeed(1450.0),
                    TurretAuto.toRightMohit,
                    Gate.gate_open,
                    FollowPath(shoot)

                ),
                Intake.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    TurretAuto.toMid,
                    FollowPath(Leave),
                    Gate.gate_close
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
    override fun onUpdate() {
    }
}


