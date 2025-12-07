package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.robot.Robot
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
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
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "blueBottom")
class blueBottom: NextFTCOpMode() {
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
        private val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0))
        private val pickUpPPGControl = Pose(73.5, 85.9, Math.toRadians(0.0))
        private val pickUpPPG2= Pose(128.9, 84.0, Math.toRadians(0.0))
        private val PPGtoShot= Pose(85.0, 16.0, Math.toRadians(119.5))
        private val PPGtoShotControl= Pose(80.0, 73.26, Math.toRadians(0.0))
    //path to pick up PGP motif
        private val pickUpPGP1 = Pose(98.25, 36.0, Math.toRadians(0.0))
        private val pickUpPGP2= Pose(135.9, 36.0, Math.toRadians(0.0))
        private val PGPtoShot= Pose(85.0, 16.0, Math.toRadians(119.5))
        private val PGPtoShotControl= Pose(87.0, 47.6, Math.toRadians(0.0))


    private val pickUpPGPcontrol= Pose(81.29, 60.7, Math.toRadians(0.0))
            //path to pick up GPP motif
        private val pickUpGPP = Pose(129.7, 28.0, Math.toRadians(0.0))
        private val pickUpGPPcontrol= Pose(82.5, 35.3, Math.toRadians(0.0))

    //PPG path chains
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    private lateinit var PPGtoShotMove: PathChain

    //PGP path chains
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain
    private lateinit var GPPtoShotMove: PathChain

    //GPP path chains
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain

    //Move a bit
    private lateinit var MoveAbit: PathChain

    private fun buildPaths() {
        //PGP paths
             PPGfirst = follower.pathBuilder()
                .addPath(BezierCurve(shootPose, pickUpPPGControl,pickUpPPG1))
                .setLinearHeadingInterpolation(shootPose.heading,pickUpPPG1.heading)
                .build()
            PPGsecond = follower.pathBuilder()
                .addPath(BezierLine(pickUpPPG1, pickUpPPG2)).setGlobalDeceleration(5.0)
                .setConstantHeadingInterpolation(0.0)
                .build()
            PPGtoShotMove = follower.pathBuilder()
                .addPath(BezierCurve(pickUpPPG2,PPGtoShotControl, PPGtoShot)).setGlobalDeceleration(4.0)
                .setLinearHeadingInterpolation(pickUpPPG2.heading,PPGtoShot.heading)
                .build()
            GPPfirst = follower.pathBuilder()
                .addPath(BezierLine(shootPose,pickUpPGP1))
                .setLinearHeadingInterpolation(shootPose.heading,pickUpPGP1.heading)
                .build()
            GPPsecond = follower.pathBuilder()
                .addPath(BezierLine(pickUpPGP1,pickUpPGP2)).setGlobalDeceleration(5.0)
                .setConstantHeadingInterpolation(0.0)
                .build()
            GPPtoShotMove = follower.pathBuilder()
                .addPath(BezierCurve(pickUpPGP2,PGPtoShotControl, PGPtoShot)).setGlobalDeceleration(4.0)
                .setLinearHeadingInterpolation(pickUpPGP2.heading, PGPtoShot.heading)
                .build()
        //PPG paths
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
                         Shooter.spinAtSpeed(1650.0),
                         Gate.gate_open,
                         Intake.spinSlowSpeed,
                         Delay(3.seconds),
            ParallelGroup(
                        Shooter.stopShooter,
                         Intake.spinStop,
                         Gate.gate_close),
            //picks up motif
        ParallelGroup(
            FollowPath(PPGfirst),
            Gate.gate_close,),
            Intake.spinFast,
            Delay(0.6.seconds),
            FollowPath(PPGsecond),
            Intake.spinStop,
            FollowPath(PPGtoShotMove),
            //shoots the motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(1685.0),
                         Gate.gate_open,
                         Intake.spinSlowSpeed,
                         Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                         Intake.spinStop,
                         Gate.gate_close),
            //picks up the non-motif
            ParallelGroup(
                FollowPath(GPPfirst),
                         Gate.gate_close,
                         Intake.spinFast),
                         Delay(0.6.seconds),
                         FollowPath(GPPsecond),
                         Intake.spinStop,
                         FollowPath(GPPtoShotMove),
            //shoots the non-motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(1725.0),
                         Gate.gate_open,
                         Intake.spinSlowSpeed,
                         Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close)
        )

    val PGP: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
            //shoots the preload
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
            //picks up motif
            FollowPath(PGPfirst),
            FollowPath(PGPsecond),
            //shoots the motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
            //picks up the non-motif
            FollowPath(GPPfirst),
            FollowPath(GPPsecond),
            //shoots the non-motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
        )

    val GPP: Command
        get() = SequentialGroup(
            FollowPath(MoveAbit),
            //shoots the preload
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
            //picks up motif
            FollowPath(GPPfirst),
            FollowPath(GPPsecond),
            //shoots the motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
            //picks up the non-motif
            FollowPath(PGPfirst),
            FollowPath(PGPsecond),
            //shoots the non-motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(-1650.0),
                         Delay(2.seconds),
        )

    override fun onInit() {
        follower.setMaxPower(1.0)
        Gate.gate_close()
        follower.setStartingPose(startPose)
        buildPaths()
    }

    override fun onStartButtonPressed() {
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


