package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.TurnBy
import dev.nextftc.extensions.pedro.TurnTo
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning
import java.lang.Compiler.command
import kotlin.concurrent.schedule
import kotlin.concurrent.timer
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@TeleOp(name = "MainTeleop")
class MainTeleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                ShooterAngle, Shooter, Gate
            ),
            BindingsComponent,
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    private val frontLeftName = "frontLeft"
    private val frontRightName = "frontRight"
    private val backLeftName = "backLeft"
    private val backRightName = "backRight"

    private val shooterController = ShooterController()

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    private lateinit var limelight: Limelight3A

    private val startPose = Pose(72.0,72.0, Math.toRadians(90.0))

    private var targetTrackingActive: Boolean = false
    private var targetTrackingCountdown: Int = 0
    private val ALIGNMENT_POWER_COARSE: Double = 0.5
    private val ALIGNMENT_POWER_FINE: Double = 0.25
    private val HEADING_TOLERANCE_FINE: Double = Math.toRadians(10.0)
    private val HEADING_TOLERANCE: Double = Math.toRadians(1.0)

    override fun onInit() {

        follower.setStartingPose(startPose)

        frontLeftMotor = MotorEx(frontLeftName)
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName)
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 11
        limelight.pipelineSwitch(1)
        limelight.start()
        follower.update()
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled.scalar = 1.0
        //driverControlled()

        button { gamepad1.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { driverControlled.scalar = 0.4 }
            .whenBecomesFalse { driverControlled.scalar = 1.0 }

        button { gamepad1.a }
            .whenBecomesTrue {
                targetTrackingActive = !targetTrackingActive
                targetTrackingCountdown = 8
            }

        button { gamepad1.x }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                intake.power = 0.7
            }
            .whenBecomesFalse {
                intake.power = 0.0
            }

        button {gamepad1.right_bumper}
            .whenBecomesTrue {
                val x = follower.pose.x
                val y = follower.pose.y
                val params = shooterController.getShot(x, y)
                if (params != null) {
                    shooterController.applyShot(params)
                } else {
                    telemetry.log().add("Shot not found for ($x, $y)")
                }
            }


        button { gamepad1.b }
            .toggleOnBecomesTrue() //make this a button command that only opens when held // default command?
            .whenBecomesTrue { Gate.gate_open() } // allow to check if gate is open on controller for comp
            .whenBecomesFalse { Gate.gate_close() }
    }

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
//        driverControlled.update()

       //start tracking goal
        val goal = Pose(16.0, 132.0)
        val x = abs(follower.pose.x)
        val y = abs(follower.pose.y)
        val diff = goal - follower.pose
        val heading = follower.heading

        val targetAngle = Math.PI - atan2(abs(diff.y), abs(diff.x))

        var headingError = targetAngle-heading
        if (headingError > Math.PI) headingError = headingError - (2 * (Math.PI))


        if (targetTrackingActive) {
            var turnPower: Double = 0.0

            if (abs(headingError) > HEADING_TOLERANCE_FINE) {
                turnPower = if (headingError > 0) {
                    -ALIGNMENT_POWER_COARSE
                } else {
                    ALIGNMENT_POWER_COARSE
                }
            } else if (abs(headingError) > HEADING_TOLERANCE) {
                turnPower = if ( headingError > 0) {
                    -ALIGNMENT_POWER_FINE
                } else {
                    ALIGNMENT_POWER_FINE
                }
            } else if (--targetTrackingCountdown > 0) {
                turnPower = if ( headingError > 0) {
                    -ALIGNMENT_POWER_FINE
                } else {
                    ALIGNMENT_POWER_FINE
                }
            } else {
                targetTrackingActive = false
            }

            val drivePower = Gamepads.gamepad1.leftStickY.get()
            val strafePower = Gamepads.gamepad1.leftStickX.get()

            frontLeftMotor.power = drivePower + strafePower + turnPower
            frontRightMotor.power = drivePower - strafePower - turnPower
            backLeftMotor.power = drivePower - strafePower + turnPower
            backRightMotor.power = drivePower + strafePower - turnPower

            telemetry.addData("Turn Power", "%.2f", turnPower)

        } else {
            // Manual Control
            driverControlled.update()
        } // end tracking goal
        telemetry.addData("X:", "%.2f", follower.pose.x)
        telemetry.addData("Y:", "%.2f", follower.pose.y)
        telemetry.addData("Heading:", "%.2f", Math.toDegrees(follower.pose.heading))
        telemetry.addData("Target: ", "%.2f", Math.toDegrees(targetAngle))
        telemetry.addData("Error: ", "%.2f", Math.toDegrees(headingError))

        telemetry.addData("Shooter Target Vel", Shooter.target)
        telemetry.addData("Shooter Actual Vel", "%.2f", Shooter.shooter.velocity)
        telemetry.addData("Angle 3Target Pos", ShooterAngle.targetPosition)
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}