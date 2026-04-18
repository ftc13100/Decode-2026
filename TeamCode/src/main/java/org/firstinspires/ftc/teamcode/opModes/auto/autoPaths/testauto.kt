
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
                blueAutoPaths, SpindexerAuto, TurretAuto
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    TurretAuto.toLeft,
                    TurretAuto.toLeft2,
                    Shooter.spinAtSpeed(1600.0),
                    FollowPath(startShoot),
                ),
                    Intake.spinFastAuto,
                    SpindexerAuto.toShoot,
                ParallelGroup(
                    TurretAuto.toLeftMore,
                    TurretAuto.toLeftMore2,
                    SpindexerAuto.toIntake,
                    FollowPath(shootPGP),
                ),
                FollowPath(PGPshoot),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toIntake,
                    FollowPath(shootGate),
                ),
                Delay(0.5.seconds),
                FollowPath(eatShoot),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,

                    SpindexerAuto.toIntake,
                    FollowPath(shootGate)),
                Delay(0.5.seconds),
                FollowPath(eatShoot),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toIntake,
                    FollowPath(shootGate),
                ),
                Delay(0.5.seconds),
                FollowPath(eatShoot),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    SpindexerAuto.toIntake,
                    FollowPath(shootPPG),
                ),
                FollowPath(PPGshoot),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinStopAuto,
                    SpindexerAuto.toIntake,
                    FollowPath(goLeave),
                ),


                    )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        //Spindexer.runToStartPos

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
        telemetry.addData("pos", "%.3f", SpindexerAuto.spindexer.currentPosition);

        telemetry.update()
    }

}
