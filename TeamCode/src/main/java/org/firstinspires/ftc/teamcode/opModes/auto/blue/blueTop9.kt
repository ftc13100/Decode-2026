package org.firstinspires.ftc.teamcode.opModes.auto.blue

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.GoToShot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.Leave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.MohitHitGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPGfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPGsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPGtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.buildPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.startPose
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "blueTop9")
class blueTop9 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }
   val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    FollowPath(GoToShot),
                    TurretAuto.toRight,
                    Gate.gate_in
                ),
                Intake.spinFast,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallShooter,
                    Intake.spinStop,
                    Gate.gate_stop
                ),
                ParallelGroup(
                    FollowPath(PPGfirst),
                    Gate.gate_stop
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
                    Gate.gate_in,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.stallShooter,
                    Intake.spinStop,
                    Gate.gate_stop
                ),
                ParallelGroup(
                    FollowPath(PGPfirst),
                    Gate.gate_stop,
                    Intake.spinFast
                ),
                FollowPath(PGPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_in,
                ),
                Intake.spinFast,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.stopShooter,
                    Gate.gate_stop,
                    FollowPath(Leave),
                    Intake.spinFast
                )
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_stop()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(startPose)
        buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }
}