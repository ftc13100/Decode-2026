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
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
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
    private lateinit var seeObelisk: PathChain
    private lateinit var goToPPG: PathChain

    private lateinit var goToPGP1: PathChain
    private lateinit var goToPGP2: PathChain


    private lateinit var PPGtoShot: PathChain





    private val startPose = Pose(126.0, 120.0, Math.toRadians(211.0))
    private val endPose = Pose(101.0, 101.0, Math.toRadians(45.0))

    private val obelisk = Pose(83.0, 83.0, Math.toRadians(90.0))
    private val pickUpPPG = Pose(128.9, 83.0, Math.toRadians(0.0))
    private val pickUpPGP1 = Pose(101.0, 59.0, Math.toRadians(0.0))
    private val pickUpPGP2 = Pose(135.9, 59.0, Math.toRadians(0.0))





    private fun buildPaths() {
        Move = follower.pathBuilder()
            .addPath(BezierLine(startPose, endPose))
            .setLinearHeadingInterpolation(startPose.heading,endPose.heading)
            .build()
        seeObelisk = follower.pathBuilder()
            .addPath(BezierLine(endPose, obelisk))
            .setLinearHeadingInterpolation(endPose.heading, obelisk.heading)
            .build()

        //PPG
        goToPPG = follower.pathBuilder()
            .addPath(BezierLine(obelisk, pickUpPPG)).setGlobalDeceleration(2.0)
            .setLinearHeadingInterpolation(obelisk.heading, pickUpPPG.heading)
            .build()

        PPGtoShot = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG, endPose))
            .setLinearHeadingInterpolation(pickUpPPG.heading, endPose.heading)
            .build()
        //PGP
        goToPGP1 = follower.pathBuilder()
            .addPath(BezierLine(endPose, pickUpPGP1)).setGlobalDeceleration(2.0)
            .setLinearHeadingInterpolation(endPose.heading, pickUpPGP1.heading)
            .build()
        goToPGP2 = follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP1, pickUpPGP2)).setGlobalDeceleration(2.0)
            .setLinearHeadingInterpolation(pickUpPGP1.heading, pickUpPGP2.heading)
            .build()

    }

//    val PPG: Command
//        get() = SequentialGroup(
//            FollowPath(Move),
//            FollowPath(seeObelisk),
//            ShooterAngle.angle_up,
//            Shooter.spinAtSpeed(1620.0),
//            Gate.gate_open,
//            Intake.spinSlowSpeed,
//            Delay(3.seconds),
//            ParallelGroup(
//                Shooter.stopShooter,
//                Intake.spinStop,
//                Gate.gate_close),
//            //picks up motif
//            ParallelGroup(
//                FollowPath(goToPPG),
//                Gate.gate_close,),
//            Intake.spinFast,
//            Delay(0.6.seconds),
//            FollowPath(PPGsecond),
//            Intake.spinStop,
//            FollowPath(PPGtoShotMove),
//            //shoots the motif
//            ShooterAngle.angle_up,
//            Shooter.spinAtSpeed(1620.0),
//            Gate.gate_open,
//            Intake.spinSlowSpeed,
//            Delay(3.seconds),
//            ParallelGroup(
//                Shooter.stopShooter,
//                Intake.spinStop,
//                Gate.gate_close),
//            //picks up the non-motif
//            ParallelGroup(
//                FollowPath(GPPfirst),
//                Gate.gate_close,
//                Intake.spinFast),
//            Delay(0.6.seconds),
//            FollowPath(GPPsecond),
//            Intake.spinStop,
//            FollowPath(GPPtoShotMove),
//            //shoots the non-motif
//            ShooterAngle.angle_up,
//            Shooter.spinAtSpeed(1620.0),
//            Gate.gate_open,
//            Intake.spinSlowSpeed,
//            Delay(3.seconds),
//            ParallelGroup(
//                Shooter.stopShooter,
//                Intake.spinStop,
//                Gate.gate_close),
//            FollowPath(MomohitPatilLeave)
//
//        )



    override fun onInit() {
        follower.setMaxPower(0.7)
        Gate.gate_close()
        follower.setStartingPose(startPose)
        buildPaths()

    }

//    override fun onStartButtonPressed() {
//        follower.setStartingPose(startPose)
//        buildPaths()
//        PoseStorage.blueAlliance = false
//        PoseStorage.redAlliance = true
//        PPG()
//    }




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


