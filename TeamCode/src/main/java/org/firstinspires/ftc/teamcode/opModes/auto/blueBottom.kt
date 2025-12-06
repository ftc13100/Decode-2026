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
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds


@Autonomous(name = "blueBottom")
class blueBottom: NextFTCOpMode() {
    init {
        addComponents(
               SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate),
                  BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

        //starting position and the pose that we will be shooting

        private val startPose = Pose(88.0, 10.0, Math.toRadians(90.0))
        private val shootPose = Pose(92.0, 14.0, Math.toRadians(119.0))
            //path to pick up PPG motif
        private val pickUpPPG = Pose(129.7, 79.0, Math.toRadians(0.0))
        private val pickUpPPGtoShotControl = Pose(72.5, 80.0, Math.toRadians(0.0))

        private val pickUpPPGcontrol= Pose(76.6, 87.0, Math.toRadians(0.0))
            //path to pick up PGP motif
        private val pickUpPGP = Pose(129.7, 55.0, Math.toRadians(0.0))
        private val pickUpPGPcontrol= Pose(81.29, 60.7, Math.toRadians(0.0))
            //path to pick up GPP motif
        private val pickUpGPP = Pose(129.7, 28.0, Math.toRadians(0.0))
        private val pickUpGPPcontrol= Pose(82.5, 35.3, Math.toRadians(0.0))

    //PPG path chains
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    private lateinit var PPGtoShoot: PathChain

    //PGP path chains
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain


    //GPP path chains
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain


    //Move a bit
    private lateinit var MoveAbit: PathChain


    private fun buildPaths() {
        //PGP paths
             PPGfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPPGcontrol,pickUpPPG)).setGlobalDeceleration(2.0)
            .setLinearHeadingInterpolation(shootPose.heading,pickUpPPG.heading)

            .build()
            PPGsecond = follower.pathBuilder()
            .addPath(BezierCurve(pickUpPPG, pickUpPPGtoShotControl, shootPose)).setGlobalDeceleration(3.0)
            .setLinearHeadingInterpolation(pickUpPPG.heading,shootPose.heading)
            .build()
        //PPG paths
            PGPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPGPcontrol,pickUpPGP)).setGlobalDeceleration(2.0)
                .setLinearHeadingInterpolation(shootPose.heading,pickUpPGP.heading)
            .build()
             PGPsecond = follower.pathBuilder()
            .addPath(BezierCurve(pickUpPGP, pickUpPGPcontrol, shootPose)).setGlobalDeceleration(5.0)
                 .setLinearHeadingInterpolation(pickUpPGP.heading,shootPose.heading)
            .build()

        //GPP paths
            GPPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpGPPcontrol,pickUpGPP))
                .setLinearHeadingInterpolation(shootPose.heading,pickUpGPP.heading)
            .build()
            GPPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP, shootPose))
                .setLinearHeadingInterpolation(pickUpGPP.heading,shootPose.heading)
            .build()

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
                         Shooter.spinAtSpeed(1750.0),
                         Gate.gate_open,
                         Intake.spinFast,
                         Delay(2.seconds),
            ParallelGroup(
                        Shooter.stopShooter,
                         Intake.spinStop,
                         Gate.gate_close),
            //picks up motif
        ParallelGroup(
            FollowPath(PPGfirst),
            Gate.gate_close,
            Intake.spinFast),
            Delay(2.seconds),
            Intake.spinStop,
            FollowPath(PPGsecond),
            //shoots the motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(1750.0),
                         Gate.gate_open,
                         Intake.spinFast,
                         Delay(2.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                         Intake.spinStop,
                         Gate.gate_close),
            //picks up the non-motif
            ParallelGroup(
                FollowPath(PGPfirst),
                         Gate.gate_close,
                         Intake.spinFast),
                         Delay(2.seconds),
                         Intake.spinStop,
                         FollowPath(PGPsecond),
            //shoots the non-motif
                         ShooterAngle.angle_up,
                         Shooter.spinAtSpeed(1650.0),
                         Gate.gate_open,
                         Intake.spinFast,
                         Delay(2.seconds),
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


