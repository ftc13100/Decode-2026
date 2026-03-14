
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import com.pedropathing.ftc.drivetrains.Mecanum
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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.TurretAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "BawlTrack")
class BawlTrack: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto, blueAutoPaths, MohitBallTrack
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val bawlTrack: Pose
        get() = Pose(9.0, (limelight.latestResult.tx + 24.0), Math.toRadians(-180.0))
    private val startPose = Pose(56.0, 24.0, Math.toRadians(-180.0))


    private lateinit var bawlRight: PathChain

    private fun buildPaths() {


        bawlRight = follower.pathBuilder()
            .addPath(BezierLine(startPose, bawlTrack))
            .setConstantHeadingInterpolation(-180.0)
            .build()
    }


    val autoRoutine: Command
        get() =
            SequentialGroup(
                FollowPath(bawlRight),
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(0.6)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(startPose)
        blueAutoPaths.buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        val result: LLResult? = limelight.latestResult
        buildPaths()
        if (result != null && result.isValid) {
            autoRoutine()
        }
    }


    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            telemetry.addData("tx", result.tx)
        }
    }
}