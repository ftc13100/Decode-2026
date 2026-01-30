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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.DeadhuzzLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.GPPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.MohitHitGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PGPtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGfirst
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGsecond
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.PPGtoShotMove
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.push
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.pushShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redTop12Push")
class redTop12Push: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto, redAutoPaths
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
                    FollowPath(push),
                    TurretAuto.toLeft,
                    Gate.gate_open
                ),
                FollowPath(pushShoot),
                Intake.spinFastAuto,
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
                Intake.spinFastAuto,
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
                Intake.spinFastAuto,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Intake.spinStop,
                    Gate.gate_close
                ),
                ParallelGroup(
                    FollowPath(PGPfirst),
                    Gate.gate_close,
                    Intake.spinFastAuto
                ),
                FollowPath(PGPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(PGPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFastAuto,
                Delay(1.8.seconds),
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    Gate.gate_close,
                    FollowPath(GPPfirst),
                    Intake.spinFastAuto
                ),
                FollowPath(GPPsecond, holdEnd = true, maxPower = 0.65),
                Intake.spinStop,
                ParallelGroup(
                    FollowPath(GPPtoShotMove),
                    ShooterAngle.angle_kindaUP,
                    Shooter.spinAtSpeed(1150.0),
                    Gate.gate_open,
                ),
                Intake.spinFastAuto,
                Delay(1.8.seconds),
                ParallelGroup(
                    FollowPath(DeadhuzzLeave),
                    Shooter.stopShooter,
                    TurretAuto.toMid,
                    Gate.gate_close,
                    Intake.spinStop
                ),
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(redAutoPaths.pushStart)
        redAutoPaths.buildPaths()
        PoseStorage.blueAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
    }
}