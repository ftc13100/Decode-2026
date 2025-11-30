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
    private var targetTrackingLoopCounter: Int = 0
    private var targetTrackingCountdown: Int = 0
    // 0.5, 0.25, 10.0, 1.0, 8 --> 2 seconds max turning
    private val ALIGNMENT_POWER_COARSE: Double = 0.6
    private val ALIGNMENT_POWER_FINE: Double = 0.2
    private val HEADING_TOLERANCE_FINE: Double = Math.toRadians(12.0)
    private val HEADING_TOLERANCE: Double = Math.toRadians(1.0)
    private val TRACKING_COUNTDOWN: Int = 12

    private var gateStatus: Boolean = false
    private var intakeStatus: Boolean = false

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
                targetTrackingCountdown = TRACKING_COUNTDOWN
                targetTrackingLoopCounter = 0
            }

        button { gamepad2.x }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                intake.power = 0.7
                intakeStatus = true
            }
            .whenBecomesFalse {
                intake.power = 0.0
                intakeStatus = false
            }

        button {gamepad1.right_bumper}
            .whenBecomesTrue {
                val x = follower.pose.x
                val y = follower.pose.y
                val params = shooterController.getShot(x, y)
                if (params != null) {
                    shooterController.applyShot(params)
                }
            }
        button {gamepad1.left_bumper}
            .whenBecomesTrue {
                CommandManager.scheduleCommand(Shooter.stopShooter())
            }


        button { gamepad2.b }
            .toggleOnBecomesTrue() //make this a button command that only opens when held // default command?
            .whenBecomesTrue {
                Gate.gate_open()
                gateStatus = true
            }
            .whenBecomesFalse {
                Gate.gate_close()
                gateStatus = false
            }
    }

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
        val llResult: LLResult? = limelight.latestResult
        var llError: Double = 99.0

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
            ++targetTrackingLoopCounter
            var turnPower: Double = 0.0
            if (targetTrackingLoopCounter == 1 && abs(headingError) < HEADING_TOLERANCE) {
                targetTrackingActive = false
            } else if (abs(headingError) > HEADING_TOLERANCE_FINE) {
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

            frontLeftMotor.power = turnPower
            frontRightMotor.power = -turnPower
            backLeftMotor.power = turnPower
            backRightMotor.power = -turnPower

        } else {
            // Manual Control
            driverControlled.update()
        } // end tracking goal

        llError = if (llResult != null && llResult.isValid) {
            llResult.tx
        } else {
            99.0
        }

        val shotParams = shooterController.getShot(x, y)

        telemetry.addData("X", "%.2f", follower.pose.x)
        telemetry.addData("Y", "%.2f", follower.pose.y)
        telemetry.addData("Heading", "%.2f", Math.toDegrees(follower.pose.heading))
        telemetry.addData("Pointing Target", "%.2f", Math.toDegrees(targetAngle))
        telemetry.addData("Pointing Error", "%.2f, %.2f", Math.toDegrees(headingError), llError)
        if (shotParams != null) {
            telemetry.addData("Shot Params", "%.0f, %.0f, %.0f, %.2f", shotParams.x, shotParams.y, shotParams.velocity,shotParams.angle)

        } else {
            telemetry.addData("Shot Params", "%.0f, %.0f, %.0f, %.2f", 0.0,0.0,0.0,0.0)
        }
        telemetry.addData("Shooter State", "%b, %b", Shooter.shooterActive, Shooter.shooterReady)
        telemetry.addData("Shooter Target / Speed", "%.2f, %.2f", Shooter.target, Shooter.shooter.velocity)
        telemetry.addData("Shooter Angle", ShooterAngle.targetPosition)
        telemetry.addData("Intake Running", intakeStatus)
        telemetry.addData("Gate Open", gateStatus)
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}