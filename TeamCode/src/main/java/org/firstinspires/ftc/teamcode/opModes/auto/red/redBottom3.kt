package org.firstinspires.ftc.teamcode.opModes.auto.red

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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.HPshoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomHP
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomIntake
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.bottomShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths.intakeShoot
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
                    Shooter.spinAtSpeed(1525.0),
                    TurretAuto.toRightMohit,
                    Gate.gate_open,
                    FollowPath(bottomShoot)

                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallShooter,
                    Intake.spinFastAuto,
                    Gate.gate_close,
                    FollowPath(bottomHP)
                ),
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Intake.spinStop,
                    Shooter.spinAtSpeed(1525.0),
                    TurretAuto.toLeftMohit,
                    Gate.gate_open,
                    FollowPath(HPshoot)

                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    Shooter.stallShooter,
                    Intake.spinFastAuto,
                    Gate.gate_close,
                    FollowPath(bottomIntake)
                ),
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Intake.spinStop,
                    Shooter.spinAtSpeed(1525.0),
                    TurretAuto.toLeftMohit,
                    Gate.gate_open,
                    FollowPath(intakeShoot)

                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    TurretAuto.toMid,
                    FollowPath(bottomLeave),
                    Gate.gate_close,
                    Intake.spinStop,

                    )

            )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(redAutoPaths.bottomStartPose)
        redAutoPaths.buildPaths()
        PoseStorage.redAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
    }
}