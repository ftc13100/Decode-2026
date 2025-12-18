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
import org.firstinspires.ftc.teamcode.subsystems.Gate
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.limelight.Vision
import org.firstinspires.ftc.teamcode.subsystems.limelight.Vision.limelight
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "blueBottomHard")
class blueBottomHard: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Vision, Shooter, Intake, Gate),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    //universal paths
    private val startPose = Pose(88.0, 9.0, Math.toRadians(90.0)).mirror()
    private val shootPose = Pose(85.0, 16.0, Math.toRadians(68.5)).mirror()
    private val getOut = Pose(96.0, 48.0, Math.toRadians(90.0)).mirror()


    //path to pick up PPG motif
    private val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpPPGControl = Pose(73.5, 85.9, Math.toRadians(0.0)).mirror()
    private val pickUpPPG2= Pose(128.9, 84.0, Math.toRadians(0.0)).mirror()
    private val PPGtoShot= Pose(85.0, 16.0, Math.toRadians(68.5)).mirror()
    private val PPGtoShotControl= Pose(80.0, 73.26, Math.toRadians(0.0)).mirror()
    //paths to pick up PGP
    private val pickUpPGP1 = Pose(96.0, 60.0, Math.toRadians(0.0)).mirror()
    private val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0)).mirror()
    private val pickUpPGP2= Pose(135.9, 60.0, Math.toRadians(0.0)).mirror()
    private val PGPtoShot= Pose(85.0, 16.0, Math.toRadians(68.5)).mirror()
    private val PGPtoShotControl= Pose(78.0, 76.0, Math.toRadians(0.0)).mirror()
    //path to pick up GPP motif
    private val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0)).mirror()
    private val pickUpGPP2= Pose(135.9, 36.0, Math.toRadians(0.0)).mirror()
    private val GPPtoShot= Pose(85.0, 16.0, Math.toRadians(68.0)).mirror()
    private val GPPtoShotControl= Pose(87.0, 47.6, Math.toRadians(0.0)).mirror()


    //PPG path chains
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    private lateinit var PPGtoShotMove: PathChain

    //PGP path chains
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain
    private lateinit var PGPtoShotMove: PathChain

    private lateinit var GPPtoShotMove: PathChain

    //GPP path chains
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain

    //Move a bit
    private lateinit var MoveAbit: PathChain
    private lateinit var MomohitPatilLeave: PathChain


    private fun buildPaths() {
        //PGP paths
        PPGfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPPGControl,pickUpPPG1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpPPG1.heading)
            .build()
        PPGsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG1, pickUpPPG2)).setGlobalDeceleration(5.0)
            .setLinearHeadingInterpolation(pickUpPPG1.heading, pickUpPPG2.heading)
            .build()
        PPGtoShotMove = follower.pathBuilder()
            .addPath(BezierCurve(pickUpPPG2,PPGtoShotControl, PPGtoShot)).setGlobalDeceleration(4.0)
            .setLinearHeadingInterpolation(pickUpPPG2.heading,PPGtoShot.heading)
            .build()
        //PGP paths
        PGPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPGPControl,pickUpPGP1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpPGP1.heading)
            .build()
        PGPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP1, pickUpPGP2)).setGlobalDeceleration(5.0)
            .setLinearHeadingInterpolation(pickUpPGP1.heading, pickUpPGP2.heading)
            .build()
        PGPtoShotMove = follower.pathBuilder()
            .addPath(BezierCurve(pickUpPGP2,PGPtoShotControl, PGPtoShot)).setGlobalDeceleration(4.0)
            .setLinearHeadingInterpolation(pickUpPGP2.heading,PGPtoShot.heading)
            .build()
        //GPP paths
        GPPfirst = follower.pathBuilder()
            .addPath(BezierLine(shootPose,pickUpGPP1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpGPP1.heading)
            .build()
        GPPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP1,pickUpGPP2)).setGlobalDeceleration(5.0)
            .setLinearHeadingInterpolation(pickUpGPP1.heading,pickUpGPP2.heading)
            .build()
        GPPtoShotMove = follower.pathBuilder()
            .addPath(BezierCurve(pickUpGPP2,GPPtoShotControl, GPPtoShot)).setGlobalDeceleration(4.0)
            .setLinearHeadingInterpolation(pickUpGPP2.heading, GPPtoShot.heading)
            .build()
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
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
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
            //picks up motif
            ParallelGroup(
                FollowPath(PGPfirst),
                Gate.gate_close,),
            Intake.spinFast,
            Delay(0.6.seconds),
            FollowPath(PGPsecond),
            Intake.spinStop,
            FollowPath(PGPtoShotMove),
            //shoots the motif
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
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
            //picks up motif
            ParallelGroup(
                FollowPath(GPPfirst),
                Gate.gate_close,),
            Intake.spinFast,
            Delay(0.6.seconds),
            FollowPath(GPPsecond),
            Intake.spinStop,
            FollowPath(GPPtoShotMove),
            //shoots the motif
            Shooter.angle_up,
            Shooter.spinAtSpeed(1620.0),
            Gate.gate_open,
            Intake.spinSlowSpeed,
            Delay(3.seconds),
            ParallelGroup(
                Shooter.stopShooter,
                Intake.spinStop,
                Gate.gate_close),
            //picks up the non-motif
            ParallelGroup(
                FollowPath(PGPfirst),
                Gate.gate_close,
                Intake.spinFast),
            Delay(0.6.seconds),
            FollowPath(PGPsecond),
            Intake.spinStop,
            FollowPath(PGPtoShotMove),
            //shoots the non-motif
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
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false

        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            for (fiducial in fiducials) {
                if (fiducial.fiducialId == 22) {
                    PGP()
                } else if (fiducial.fiducialId == 23) {
                    PPG()
                } else if (fiducial.fiducialId == 21) {
                    GPP()
                } else {
                    GPP()
                }
            }
        } else {
            GPP()
        }
    }

    override fun onUpdate() {
        telemetry.addData("Mode", "TeleOp Running")

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
        telemetry.addData("X", "%.2f, Y: %.2f, Heading: %.2f", follower.pose.x, follower.pose.y, Math.toDegrees(follower.pose.heading))
        telemetry.addData("Turret Pos", "Current: %.0f, Target: %.0f, Start: %.0f, Left: %.0f, Right: %.0f",  Turret.turret.currentPosition, Turret.target,
            Turret.startPosition,
            Turret.leftLimit,
            Turret.rightLimit)
        telemetry.addData("Turret", "Active: %b, Ready: %b, ReadyMs: %.0f, GoalTracking: %b, Power: %.2f",Turret.turretActive, Turret.turretReady, Turret.turretReadyMs, Turret.goalTrackingActive,
            Turret.turret.power)
        telemetry.addData("Shooter Speed", "Current: %.0f, Target: %.0f", Shooter.shooter.velocity, Shooter.target)
        telemetry.addData("Shooter", "Ready: %b, ReadyMs:  %.0f, Active: %b, Power: %.2f", Shooter.shooterReady, Shooter.shooterReadyMs, Shooter.shooterActive, Shooter.shooter.power)


        telemetry.update()
    }
//     override fun onStop() {
//    }
}


