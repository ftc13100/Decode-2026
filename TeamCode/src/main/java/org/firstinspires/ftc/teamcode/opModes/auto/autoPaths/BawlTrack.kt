
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

import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "testauto")
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

    val bawlTrack = Pose(9.0, (limelight.latestResult.tx - 24.0), Math.toRadians(180.0))
    val startPose = Pose(56.0, 24.0, Math.toRadians(180.0))


    lateinit var  bawlRight : PathChain

    fun buildPaths() {
        bawlRight= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(startPose, bawlTrack))
            .setConstantHeadingInterpolation(180.0)
            .build()
    }


    val autoRoutine: Command
        get() =
            SequentialGroup(
                FollowPath(bawlRight),
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(blueAutoPaths.start)
        blueAutoPaths.buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
                autoRoutine()
            }
        }



    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
        limelight.latestResult
    }


}