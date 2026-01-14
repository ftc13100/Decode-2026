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
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.GPPfirst
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.GPPsecond
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.GoToShot
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.Leave
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.MohitHitGate
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PGPfirst
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PGPsecond
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PGPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PPGfirst
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PPGsecond
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.PPGtoShotMove
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.TheGate
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.buildPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.blueAutoPaths.startPose

@Autonomous(name = "blueTop12")
class blueTop12: NextFTCOpMode() {
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
                Delay(2.3.seconds),
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
                Delay(1.8.seconds),
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
                Delay(1.8.seconds),
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
                Delay(1.8.seconds),
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

