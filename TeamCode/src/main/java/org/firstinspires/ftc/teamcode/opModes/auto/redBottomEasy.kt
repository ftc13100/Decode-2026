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
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.limelight.Vision
import org.firstinspires.ftc.teamcode.subsystems.limelight.Vision.limelight
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redBottomEasy")
class redBottomEasy: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Vision, Shooter, Intake, Gate),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //starting position and the pose that we will be shooting

    private val startPose = Pose(88.0, 9.0, Math.toRadians(90.0))
    private val shootPose = Pose(85.0, 16.0, Math.toRadians(68.5))
    private val getOut = Pose(96.0, 48.0, Math.toRadians(90.0))

    //path to pick up PPG motif
    //PPG path chains

    //Move a bit
    private lateinit var MoveAbit: PathChain
    private lateinit var MomohitPatilLeave: PathChain


    private fun buildPaths() {
        //PGP paths

        //Move a bit
        MoveAbit = follower.pathBuilder()
            .addPath(BezierLine(startPose,shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        MomohitPatilLeave = follower.pathBuilder()
            .addPath(BezierLine(shootPose,getOut))
            .setLinearHeadingInterpolation(shootPose.heading, getOut.heading)
            .build()
    }

    val PPG: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
            //shoots the preload
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
            FollowPath(MomohitPatilLeave)

        )

    val PGP: Command
        get() = SequentialGroup( FollowPath(MoveAbit),
            //shoots the preload
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
            FollowPath(MomohitPatilLeave)

        )

    val GPP: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
            //shoots the preload
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
            FollowPath(MomohitPatilLeave)

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


