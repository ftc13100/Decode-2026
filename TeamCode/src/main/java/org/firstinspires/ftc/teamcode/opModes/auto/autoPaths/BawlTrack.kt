
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import SpindexerAuto
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.TurretAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower


@Autonomous(name = "BawlTrack")
class BawlTrack: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitBallTrack, Shooter, ShooterAngle, Intake, PoseStorage,
                blueAutoPaths, SpindexerAuto, TurretAuto, NewTurret
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val limelight: Limelight3A by lazy {
        hardwareMap.get(
            Limelight3A::class.java,
            "limelight"
        )
    }

    val start = Pose(60.21342512908778, 24.11876075731497, Math.toRadians(180.0))

    lateinit var startShoot: PathChain


    override fun onInit() {
        telemetry.setMsTransmissionInterval(11)
        limelight.pipelineSwitch(8)
        limelight.start()
    }



    override fun onStartButtonPressed() {
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val tx = result.getTx()
            val bawl = Pose(9.5, tx, Math.toRadians(180.0))

            fun buildPaths() {
                startShoot = PedroComponent.Companion.follower.pathBuilder()
                    .addPath(BezierLine(start, bawl))
                    .setConstantHeadingInterpolation(start.heading)
                    .build()
            }

        }



    }

    override fun onUpdate() {

        telemetry.update()
    }

}