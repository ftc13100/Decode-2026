package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
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
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redBottomHard")
class redBottomHard: NextFTCOpMode() {
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
        private val startPose = Pose(125.23, 121.52, Math.toRadians(37.0))
        private val shootPose = Pose(78.0, 78.0, Math.toRadians(0.0))
        private val leavePoint = Pose(96.0, 48.0, Math.toRadians(0.0))
        private val hitGate = Pose(127.4, 73.65, Math.toRadians(0.0))
        private val hitGateControl = Pose(93.118, 77.804, Math.toRadians(0.0))
    //path to pick up PPG motif
        private val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0))
        private val pickUpPPGControl = Pose(73.5, 85.9, Math.toRadians(0.0))
        private val pickUpPPG2= Pose(127.2, 84.0, Math.toRadians(0.0))
        private val PPGtoShot= Pose(78.0, 78.0, Math.toRadians(0.0))
    //paths to pick up PGP
        private val pickUpPGP1 = Pose(96.0, 60.0, Math.toRadians(0.0))
        private val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0))
        private val pickUpPGP2= Pose(133.3, 60.0, Math.toRadians(0.0))
        private val PGPtoShot= Pose(78.0, 78.0, Math.toRadians(0.0))
        private val PGPtoShotControl= Pose(77.485, 53.68, Math.toRadians(0.0))
    //path to pick up GPP motif
        private val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0))
        private val pickUpGPP2= Pose(133.3, 36.0, Math.toRadians(0.0))
        private val GPPtoShot= Pose(78.0, 78.0, Math.toRadians(0.0))
        private val GPPtoShotControl= Pose(87.0, 47.6, Math.toRadians(0.0))


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
    private lateinit var PlsHitGate: PathChain

    private fun buildPaths() {
        //Universal Paths
            GoToShot = follower.pathBuilder()
                .addPath(BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
                .build()
            Leave = follower.pathBuilder()
                .addPath(BezierLine(PGPtoShot,leavePoint))
                .setLinearHeadingInterpolation(shootPose.heading, leavePoint.heading)
                .build()
            PlsHitGate = follower.pathBuilder()
                .addPath(BezierCurve(pickUpPPG2, hitGateControl,  hitGate))
                .setLinearHeadingInterpolation(pickUpPPG2.heading,hitGate.heading)
                .build()
        //PPG paths
             PPGfirst = follower.pathBuilder()
                .addPath(BezierCurve(PGPtoShot, pickUpPPGControl,pickUpPPG1))
                .setLinearHeadingInterpolation(shootPose.heading,pickUpPPG1.heading)
                .build()
            PPGsecond = follower.pathBuilder()
                .addPath(BezierLine(pickUpPPG1, pickUpPPG2))
                .setConstantHeadingInterpolation(0.0)
                .build()
            PPGtoShotMove = follower.pathBuilder()
                .addPath(BezierCurve(pickUpPPG2, PPGtoShot))
                .setLinearHeadingInterpolation(pickUpPPG2.heading,PPGtoShot.heading)
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
                .addPath(BezierCurve(pickUpGPP2,GPPtoShotControl, GPPtoShot))
                .setLinearHeadingInterpolation(pickUpGPP2.heading, GPPtoShot.heading)
                .build()
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1360.0),
                    FollowPath(GoToShot),
                    TurretAuto.toLeft,
                    Gate.gate_open
                ),
                Intake.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
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
                ParallelGroup(
                FollowPath(PPGtoShotMove),
                ShooterAngle.angle_kindaUP,
                Shooter.spinAtSpeed(1360.0),
                Gate.gate_open,
                             ),
                                Intake.spinFast,
                                Delay(2.3.seconds),
                ParallelGroup(
                Shooter.stopShooter,
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
                Shooter.spinAtSpeed(1360.0),
                Gate.gate_open,
                             ),
                                Intake.spinFast,
                                Delay(2.3.seconds),
                ParallelGroup(
                Shooter.stopShooter,
                Gate.gate_close,
                FollowPath(GPPfirst),
                Intake.spinFast
                             ),
                                FollowPath(GPPsecond, holdEnd = true, maxPower = 0.65),
                                Intake.spinStop,
                ParallelGroup(
                FollowPath(GPPtoShotMove),
                ShooterAngle.angle_kindaUP,
                Shooter.spinAtSpeed(1370.0),
                Gate.gate_open,
                             ),
                                Intake.spinFast,
                                Delay(2.3.seconds),
                ParallelGroup(
                Shooter.stopShooter,
                Gate.gate_close,
                FollowPath(Leave),
                Intake.spinStop),
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
        autoRoutine()
    }

    override fun onUpdate() {
    }
}


