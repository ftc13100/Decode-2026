package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.control.feedforward.FeedforwardElement
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.telemetry
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterTable.shooterLookupTable
import java.time.Instant

@Configurable
object Shooter : Subsystem {
    @JvmField var target = 0.0
    var servoTargetPosition = 0.0
    @JvmField var velPIDCoefficients = PIDCoefficients(0.00375, 0.0, 0.0)
    @JvmField var basicFFParameters = BasicFeedforwardParameters(0.00048, 0.0004, 0.000145)

    val servo = ServoEx("angle")
    val shooter = MotorEx("shooter").brakeMode().reversed()

    var shooterActive  = false
    var shooterReady  = false
    var shooterReadyMs: Double = 0.00
    private val runtime = ElapsedTime()

    val controller = controlSystem {
        velPid(velPIDCoefficients)
        basicFF(basicFFParameters)
    }

    override fun periodic() {
        if (shooterActive){
        shooter.power = controller.calculate(
            shooter.state
        )
        } else {
            shooter.power = 0.0
        }
    }

    fun spinAtSpeed(speed: Double) =
        InstantCommand {
            target = speed
            shooterActive = true
            shooterReady = false
            shooterReadyMs = 0.00
            runtime.reset()
        }.then(
            RunToVelocity(controller, speed, 11.0),
            InstantCommand {
                shooterReady = true
                shooterReadyMs = runtime.milliseconds()
            }
        ).setInterruptible(true).requires(this)

    val stopShooter =
        InstantCommand {
            shooterActive = false
            shooterReady = false
            target = 0.0
        }.requires(this)



    fun spinning() {
        shooterActive = true
        controller.goal = KineticState(velocity = target)
        shooter.power = controller.calculate(
            KineticState(velocity = shooter.velocity)
        )
    }

    val update = InstantCommand { servo.position = servoTargetPosition }

    val angle_up = InstantCommand {
        servo.position = 0.5
    }

    val angle_middle = InstantCommand {
        servo.position = 0.6
    }

    val angle_down = InstantCommand {
        servo.position = 0.7
    }

    fun toAngle (angle : Double) =
        InstantCommand {
            servo.position = angle
        }

    fun getShot(x: Double, y: Double) =
        shooterLookupTable
            .entries
            .find { x - it.key.first <= 6.0 && y - it.key.second <= 6.0 }
            ?.value

    fun applyShot(velocity: Double, angle: Double) {
        servoTargetPosition = angle
        update()

        CommandManager.scheduleCommand(
            spinAtSpeed(velocity)
        )
    }
}