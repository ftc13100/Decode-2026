package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.telemetry
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
@Configurable
object Shooter : Subsystem {
    @JvmField var target = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.05, 0.0, 0.4)

    val shooter = MotorEx("shooter").brakeMode()
    var shooterActive  = false
    var shooterReady  = false

    val controller = controlSystem {
        velPid(velPIDCoefficients)
    }

    fun spinAtSpeed(speed: Double) =
        InstantCommand{
            shooterActive = true
            shooterReady = false
        }.then(
                RunToVelocity(controller, speed, 5.0),
                InstantCommand { shooterReady = true }
            ).setInterruptible(true).requires(this)

    override fun periodic() {
        if (shooterActive){
        shooter.power = controller.calculate(
            shooter.state
        )
        } else {
            shooter.power = 0.0
        }
    }

    fun stopShooter() {
        shooterActive = false
        shooterReady = false
    }



    fun spinning() {
        controller.goal = KineticState(velocity = target)
        shooter.power = controller.calculate(
            KineticState(velocity = shooter.velocity)
        )
    }
}