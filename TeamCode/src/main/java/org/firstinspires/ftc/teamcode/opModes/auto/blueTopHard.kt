package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
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
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "blueTopHard")
class blueTopHard: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }
    ////////KEY//////////
    //pickUp_____1 == Getting to pick up motif row
    //pickUp_____2 == Ramming into the motif
    //___toShot    == Goes to the shootPose after picking up
    //_____Control == Dilates the straight line into a curve

    ///////DOCS//////////
    //Parallel Groups    == Everything within happens at the same time
    //Sequential Groups  == Everything within happens chronologically


    //universal paths
    private val startPose = Pose(125.23, 121.52, Math.toRadians(37.0)).mirror()
    private val shootPose = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val leavePoint = Pose(84.0, 110.0, Math.toRadians(-16.5)).mirror()

    private val wigglePoint = Pose(87.0, 113.0, Math.toRadians(-16.5)).mirror()
    private val gate = Pose(127.5, 75.90674955595027, Math.toRadians(125.0)).mirror()

    //path to pick up PPG motif
    private val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpPPG2= Pose(127.2, 84.0, Math.toRadians(0.0)).mirror()
    private val PPGtoShot= Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    //paths to pick up PGP
    private val pickUpPGP1 = Pose(96.0, 60.0, Math.toRadians(0.0)).mirror()
    private val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0)).mirror()
    private val pickUpPGP2= Pose(133.3, 60.0, Math.toRadians(0.0)).mirror()
    private val PGPtoShot= Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val PGPtoShotControl= Pose(77.485, 53.68, Math.toRadians(0.0)).mirror()
    //path to pick up GPP motif
    private val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0)).mirror()
    private val pickUpGPP2= Pose(133.3, 36.0, Math.toRadians(0.0)).mirror()
    private val GPPtoShot= Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val GPPtoShotControl= Pose(87.0, 47.6, Math.toRadians(0.0)).mirror()


    //PPG path chains
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    private lateinit var PPGtoShotMove: PathChain

    //PGP path chains
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain
    private lateinit var PGPtoShotMove: PathChain

    //GPP path chains
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain
    private lateinit var GPPtoShotMove: PathChain

    //Move a bit
    private lateinit var GoToShot: PathChain
    private lateinit var Leave: PathChain
    private lateinit var MohitHitGate: PathChain
    private lateinit var Wiggle: PathChain



    private fun buildPaths() {
        //Universal Paths
        GoToShot = follower.pathBuilder()
            .addPath(BezierLine(startPose,shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        Leave = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP2,leavePoint))
            .setLinearHeadingInterpolation(shootPose.heading, leavePoint.heading)
            .build()
        MohitHitGate = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG2, gate))
            .setLinearHeadingInterpolation(pickUpPPG2.heading,gate.heading)
            .build()
        Wiggle = follower.pathBuilder()
            .addPath(BezierLine(shootPose, wigglePoint))
            .setLinearHeadingInterpolation(shootPose.heading,wigglePoint.heading)
            .addPath(BezierLine(wigglePoint, shootPose))
            .setLinearHeadingInterpolation(shootPose.heading,wigglePoint.heading)
            .addPath(BezierLine(shootPose, wigglePoint))
            .setLinearHeadingInterpolation(shootPose.heading,wigglePoint.heading)
            .addPath(BezierLine(wigglePoint, shootPose))
            .setLinearHeadingInterpolation(shootPose.heading,wigglePoint.heading)
            .build()
        //PPG paths
        PPGfirst = follower.pathBuilder()
            .addPath(BezierLine(shootPose, pickUpPPG1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpPPG1.heading)
            .build()
        PPGsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG1, pickUpPPG2))
            .setConstantHeadingInterpolation(0.0)
            .build()
        PPGtoShotMove = follower.pathBuilder()
            .addPath(BezierLine(gate, PPGtoShot))
            .setLinearHeadingInterpolation(gate.heading,PPGtoShot.heading)
            .build()
        //PGP paths
        PGPfirst = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPGPControl,pickUpPGP1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpPGP1.heading)
            .build()
        PGPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP1, pickUpPGP2))
            .setConstantHeadingInterpolation(0.0)
            .build()
        PGPtoShotMove = follower.pathBuilder()
            .addPath(BezierCurve(pickUpPGP2,PGPtoShotControl, PGPtoShot))
            .setLinearHeadingInterpolation(pickUpPGP2.heading,PGPtoShot.heading)
            .build()
        //GPP paths
        GPPfirst = follower.pathBuilder()
            .addPath(BezierLine(shootPose,pickUpGPP1))
            .setLinearHeadingInterpolation(shootPose.heading,pickUpGPP1.heading)
            .build()
        GPPsecond = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP1,pickUpGPP2))
            .setConstantHeadingInterpolation(0.0)
            .build()
        GPPtoShotMove = follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP2, GPPtoShot))
            .setLinearHeadingInterpolation(pickUpGPP2.heading, GPPtoShot.heading)
            .build()
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    FollowPath(GoToShot),
                    TurretAuto.toRight,
                    Gate.gate_open
                ),
                Intake.spinFast,
                Delay(1.seconds),
                FollowPath(Wiggle),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(PPGfirst),
                    Gate.gate_close
                ),
                Intake.spinFast,
                FollowPath(PPGsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                FollowPath(MohitHitGate),
                Delay(1.0.seconds),
                ParallelGroup(
                    FollowPath(PPGtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(0.6.seconds),
                FollowPath(Wiggle),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(PGPfirst),
                    Gate.gate_close,
                    Intake.spinFast
                ),
                FollowPath(PGPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(0.6.seconds),
                FollowPath(Wiggle),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Gate.gate_close,
                    FollowPath(GPPfirst),
                    Intake.spinFast
                ),
                FollowPath(GPPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(Leave),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFast,
                Delay(0.6.seconds),
                FollowPath(Wiggle),
                ParallelGroup(
                    Shooter.stopShooter,
                    TurretAuto.toMid,
                    Gate.gate_close,
                    Intake.spinStop),
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
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = follower.pose
    }

    override fun onUpdate() {
    }
}


