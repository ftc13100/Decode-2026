
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import SpindexerAuto
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPshoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPGshoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.eatShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.gateEat
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.goLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootPGP
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootPPG
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.startShoot
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.TurretAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "testauto")
class testauto: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Shooter, ShooterAngle, Intake, PoseStorage,
                blueAutoPaths, SpindexerAuto
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    Shooter.spinAtSpeed(1000.0),
                    FollowPath(startShoot),
                ),
                ParallelRaceGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toShoot
                ),
                SpindexerAuto.toIntake,
                ParallelGroup(
                FollowPath(shootPGP),
                    Intake.spinFast
                ),
                Intake.spinStopAuto,
                FollowPath(PGPshoot),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                SpindexerAuto.toIntake,
                FollowPath(shootGate),
                ParallelGroup(
                FollowPath(gateEat),
                    Intake.spinFastAuto,
                ),
                Delay(1.seconds),
                Intake.spinStopAuto,
                FollowPath(eatShoot),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                FollowPath(shootGate),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                FollowPath(eatShoot),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                SpindexerAuto.toIntake,
                FollowPath(shootGate),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                FollowPath(eatShoot),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                SpindexerAuto.toIntake,
                ParallelGroup(
                    Intake.spinFastAuto,
                FollowPath(shootPPG)
                ),
                Intake.spinStopAuto,
               FollowPath(PPGshoot),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                Intake.spinStopAuto,
                SpindexerAuto.toIntake,
                FollowPath(goLeave)
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(blueAutoPaths.start)
        blueAutoPaths.buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
        telemetry.addData("Velocity", "%.3f", Shooter.shooter.velocity);

        telemetry.update()
    }

}
