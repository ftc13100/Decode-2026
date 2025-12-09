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

@Autonomous(name = "test")
class test: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //starting position and the pose that we will be shooting

    private val startPose = Pose(19.0, 123.0, Math.toRadians(323.0))
    private val shootPose = Pose(72.0, 72.0, Math.toRadians(90.0))
    //path to pick up PPG motif



    //Move a bit
    private lateinit var MoveAbit: PathChain

    private fun buildPaths() {
        //PGP paths

        //Move a bit
        MoveAbit = follower.pathBuilder()
            .addPath(BezierLine(startPose,shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
    }

    val PPG: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
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
        PPG()
    }

    override fun onUpdate() {
        telemetry.addData("Shooter Speed", "Current: %.0f, Target: %.0f", Shooter.shooter.velocity, Shooter.target)
        telemetry.addData("", "X: %.0f, Y: %.0f, Heading: %.1f, Target: %.1f", follower.pose.x, follower.pose.y, Math.toDegrees(follower.pose.heading))


        val result: LLResult? = limelight.latestResult

        PoseStorage.poseEnd = follower.pose

        // Common Telemetry


        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }
//     override fun onStop() {
//    }
}


