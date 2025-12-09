package org.firstinspires.ftc.teamcode.opModes.auto

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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds


@Autonomous(name = "redTopHard")
class redTopHard: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //starting position and the pose that we will be shooting


    private lateinit var Move: PathChain

    private val startPose = Pose(96.9, 83.7, Math.toRadians(0.0))
    private val endPose = Pose(136.44, 84.0, Math.toRadians(0.0))

    private fun buildPaths() {
        Move = follower.pathBuilder()
            .addPath(BezierLine(startPose, endPose)).setGlobalDeceleration(2.0)
            .setConstantHeadingInterpolation(startPose.heading)
            .build()
    }

    val PPG: Command
        get() = SequentialGroup(
            Gate.gate_close,
            ParallelGroup(
            FollowPath(Move),
                Intake.spinFast),
            Delay(0.5.seconds),
            Intake.spinStop

            )



    override fun onInit() {
        follower.setMaxPower(0.7)
        Gate.gate_close()
        follower.setStartingPose(startPose)
        buildPaths()

    }

    override fun onStartButtonPressed() {
        PPG()
//        val result: LLResult? = limelight.latestResult
//        if (result != null && result.isValid) {
//            val fiducials = result.fiducialResults
//            for (fiducial in fiducials) {
//                if (fiducial.fiducialId == 22) {
//                    PPG() }
//            }
//        }

    }




    override fun onUpdate() {
        telemetry.addData("Shooter Speed", "Current: %.0f, Target: %.0f", Shooter.shooter.velocity, Shooter.target)


        val result: LLResult? = limelight.latestResult

        // Common Telemetry
        if (result != null && result.isValid) {
            val botpose: Pose3D = result.botpose
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
            telemetry.addData("Bot pose", botpose.toString())
        } else {
            telemetry.addData("Limelight", "Target not found")
        }

        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }   }


