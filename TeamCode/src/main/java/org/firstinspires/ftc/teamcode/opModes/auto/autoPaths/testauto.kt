
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGP
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPback
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPcontrol
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PGPshoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPG
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.PPGshoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.eat
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.eatShoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.gate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.gateBack
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.gateEat
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.goLeave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.leave
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shoot
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootGate
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootPGP
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.shootPPG
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.blueAutoPaths.startShoot
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
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
                TurretAuto, blueAutoPaths
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                Shooter.spinAtSpeed(1500.0),
                ParallelGroup(
                    TurretAuto.toLeft,
                    TurretAuto.toLeft2
                ),
                ParallelGroup(
                TurretAuto.toRight,
                TurretAuto.toRight2
                ),
                FollowPath(startShoot),
                ParallelGroup(
                    Intake.spinFastAuto,
                FollowPath(shootPGP)
            ),
                FollowPath(PGPshoot),
                FollowPath(shootGate),
                ParallelGroup(
                    Intake.spinFastAuto,
                FollowPath(gateEat)),
                Delay(1.seconds),
                FollowPath(eatShoot),
                FollowPath(shootGate),
                ParallelGroup(
                    Intake.spinFastAuto,
                    FollowPath(gateEat)),
                Delay(1.seconds),
                FollowPath(eatShoot),
                FollowPath(shootGate),
                ParallelGroup(
                    Intake.spinFastAuto,
                    FollowPath(gateEat)),
                Delay(1.seconds),
                FollowPath(eatShoot),
                ParallelGroup(
                    Intake.spinFastAuto,
                    FollowPath(gateEat)),
                FollowPath(PPGshoot),
                FollowPath(goLeave)
            )

    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(blueAutoPaths.start)
        blueAutoPaths.buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        autoRoutine()
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            for (fiducial in fiducials) {
                if (fiducial.fiducialId == 22) {
                   }
                else if (fiducial.fiducialId == 23) {

                } else {

                }

            }
        }
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
