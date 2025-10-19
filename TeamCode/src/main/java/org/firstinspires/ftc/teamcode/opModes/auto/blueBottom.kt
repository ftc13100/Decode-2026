package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil.limelight
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@Autonomous(name = "blueBottom")
class blueBottom: NextFTCOpMode() {
    init {
        addComponents(
               SubsystemComponent(MohitPatil),
                  BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

        //starting position and the pose that we will be shooting
    private val shootPose = Pose(88.0, 8.0, Math.toRadians(90.0))
            //path to pick up PPG motif
        private val pickUpPPG = Pose(127.7, 83.0, Math.toRadians(0.0))
        private val pickUpPPGcontrol= Pose(76.6, 91.0, Math.toRadians(0.0))
            //path to pick up PGP motif
        private val pickUpPGP = Pose(127.7, 59.0, Math.toRadians(0.0))
        private val pickUpPGPcontrol= Pose(79.29, 64.7, Math.toRadians(0.0))
            //path to pick up GPP motif
        private val pickUpGPP = Pose(127.7, 59.0, Math.toRadians(0.0))
        private val pickUpGPPcontrol= Pose(82.5, 39.3, Math.toRadians(0.0))
            //path to pick up from the human player
    private val pickUPhp= Pose(140.0,8.2,Math.toRadians(0.0))

    //PPG path chains
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    //PGP path chains
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain

    //GPP path chains
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain

    //Universal path chains
    private lateinit var HumanPlayer: PathChain
    private lateinit var ShootAgain: PathChain

    private fun buildPaths() {
        //PGP paths
             PPGfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPPGcontrol,pickUpPPG))
            .setConstantHeadingInterpolation(0.0)
            .build()
            PPGsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG, shootPose))
            .setConstantHeadingInterpolation(0.0)
            .build()
        //PPG paths
            PGPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPGPcontrol,pickUpPGP))
            .setConstantHeadingInterpolation(0.0)
            .build()
             PGPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP, shootPose))
            .setConstantHeadingInterpolation(0.0)
            .build()
        //GPP paths
            GPPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpGPPcontrol,pickUpGPP))
            .setConstantHeadingInterpolation(0.0)
            .build()
            GPPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP, shootPose))
            .setConstantHeadingInterpolation(0.0)
            .build()
        //Universal paths
             HumanPlayer= follower.pathBuilder()
            .addPath(BezierLine(shootPose, pickUPhp))
            .setConstantHeadingInterpolation(0.0)
            .build()
             ShootAgain= follower.pathBuilder()
            .addPath(BezierLine(pickUPhp,shootPose))
            .setConstantHeadingInterpolation(0.0)
            .build()
    }

    val PPG: Command
        get() = SequentialGroup(

            FollowPath(PPGfirst),
                         FollowPath(PPGsecond),
                         FollowPath(HumanPlayer),
                         FollowPath(ShootAgain)


        )

    val PGP: Command
        get() = SequentialGroup(
            FollowPath(PGPfirst),
                         FollowPath(PGPsecond),
                         FollowPath(HumanPlayer),
                         FollowPath(ShootAgain)

        )
    val GPP: Command
        get() = SequentialGroup(
            FollowPath(GPPfirst),
                         FollowPath(GPPsecond),
                         FollowPath(HumanPlayer),
                         FollowPath(ShootAgain)

        )

    override fun onInit() {
        follower.setMaxPower(0.7)
        follower.setStartingPose(shootPose)
        buildPaths()
    }

    override fun onStartButtonPressed() {
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            for (fiducial in fiducials) {
                if (fiducial.fiducialId == 22) {
                    PGP()
                    if (fiducial.fiducialId == 23) {
                        PPG()
                    } else if (fiducial.fiducialId == 21) {
                        GPP()
                    }

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
    }    }


