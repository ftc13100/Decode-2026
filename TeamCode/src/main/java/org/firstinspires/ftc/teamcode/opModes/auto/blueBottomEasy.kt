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

@Autonomous(name = "blueBottomEasy")
class blueBottomEasy: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //starting position and the pose that we will be shooting

    private val startPose = Pose(88.0, 9.0, Math.toRadians(90.0))
    private val shootPose = Pose(85.0, 16.0, Math.toRadians(121.0))
    //path to pick up PPG motif


    private val pickUpPGPcontrol= Pose(81.29, 60.7, Math.toRadians(0.0))
    //path to pick up GPP motif
    private val pickUpGPP = Pose(129.7, 28.0, Math.toRadians(0.0))
    private val pickUpGPPcontrol= Pose(82.5, 35.3, Math.toRadians(0.0))

    //PPG path chains

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
            //shoots the preload
            ShooterAngle.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
        )

    val PGP: Command
        get() = SequentialGroup( FollowPath(MoveAbit),
            //shoots the preload
            ShooterAngle.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
        )

    val GPP: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
            //shoots the preload
            ShooterAngle.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
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

        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            for (fiducial in fiducials) {
                if (fiducial.fiducialId == 22) {
                    PGP() }
                else if (fiducial.fiducialId == 23) {
                    PPG()
                } else {
                    GPP()
                }
            }
        }
    }

    override fun onUpdate() {
        telemetry.addData("Shooter Speed", "Current: %.0f, Target: %.0f", Shooter.shooter.velocity, Shooter.target)

        val result: LLResult? = limelight.latestResult

        PoseStorage.poseEnd = follower.pose

        // Common Telemetry
        if (result != null && result.isValid) {
            // val botpose: Pose3D = result.botpose
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
            //    telemetry.addData("Bot pose", botpose.toString())
        } else {
            telemetry.addData("Limelight", "Target not found")
        }

        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }
//     override fun onStop() {
//    }
}


