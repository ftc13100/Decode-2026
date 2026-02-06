package org.firstinspires.ftc.teamcode.opModes.auto.red

import com.pedropathing.ftc.drivetrains.Mecanum
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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.HPviggle
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.HPviggleagain
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.HPviggletoShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomHP
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomIntake
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomIntake2
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomIntake2toShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomSpikeGet
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomSpikeGetBack
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.intakeShoot

import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redBottom3")
class redBottom3 : NextFTCOpMode() {
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
                    ShooterAngle.angle_up,
                    Shooter.spinAtSpeed(1450.0),
                    TurretAuto.toLeftMohitFar,
                    Gate.gate_open,
                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallerShooterFar,
                    Intake.spinFastAuto,
                    Gate.gate_close,
                    FollowPath(bottomHP)
                ),
                FollowPath(HPviggle),
                FollowPath(HPviggleagain),
                ParallelGroup(
                    Intake.spinStop,
                    Gate.gate_close,
                ),
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Intake.spinStop,
                    Shooter.spinAtSpeed(1450.0),
                    Gate.gate_open,
                    FollowPath(HPviggletoShoot)
                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallerShooterFar,
                    Intake.spinFastAuto,
                    Gate.gate_close,
                    FollowPath(intakeShoot)
                ),
                FollowPath(bottomIntake),
                FollowPath(bottomIntake2),
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Intake.spinStop,
                    Shooter.spinAtSpeed(1450.0),
                    Gate.gate_open,
                    FollowPath(bottomIntake2toShoot)
                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallerShooterFar,
                    Intake.spinFastAuto,
                    Gate.gate_close,
                    FollowPath(bottomHP)
                ),
                FollowPath(HPviggle),
                FollowPath(HPviggleagain),
                ParallelGroup(
                    Intake.spinStop,
                    Gate.gate_close,
                ),
                ParallelGroup(
                    TurretAuto.toMid,
                    FollowPath(bottomLeave),
                    Gate.gate_close,
                    Intake.spinStop,
                    Shooter.stopShooter

                )
            )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(redAutoPaths.bottomStartPose)
        redAutoPaths.buildPaths()
        PoseStorage.blueAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
        val dt = follower.drivetrain as Mecanum
        val powers = dt.motors.map { it.power }
        telemetry.addData("Power", powers)
        telemetry.update()
    }

}