package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.TurnTo
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "sachetTest")
class sachetTest: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //starting position and the pose that we will be shooting

    private val startPose = Pose(88.0, 9.0, Math.toRadians(0.0))
    private val sachetPose = Pose(88.0, 9.05, Math.toRadians(90.0))



    //path to pick up PPG motif
    //PPG path chains

    //Move a bit
    private lateinit var SachetMove: PathChain



    private fun buildPaths() {
        //PGP paths

        //Move a bit
        SachetMove = follower.pathBuilder()
            .addPath(BezierLine(startPose,sachetPose))
            .setLinearHeadingInterpolation(startPose.heading,sachetPose.heading)
            .build()

    }

    val PPG: Command
        get() = SequentialGroup(
            FollowPath(SachetMove)
        )



    override fun onInit() {
        follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(startPose)
        buildPaths()
        PPG()

        }
    }





