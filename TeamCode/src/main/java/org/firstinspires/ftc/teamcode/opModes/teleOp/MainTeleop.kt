package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.opencv.core.Mat
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.time.Duration.Companion.seconds
@TeleOp(name = "MainTeleop")
class MainTeleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                ShooterAngle, Shooter, Gate, Intake, Turret, PoseStorage
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

    private val shooterController = ShooterController

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

//    private val imu = IMUEx("imu", Direction.LEFT, Direction.UP).zeroed()

    private lateinit var limelight: Limelight3A
    private val startPose = PoseStorage.poseEnd  //(72.0,72.0, Math.toRadians(90.0))
    private val testingPose = Pose(72.0,72.0,Math.toRadians(90.0))
    private var testMode: Boolean = false
    private var targetTrackingActive: Boolean = false
    private var targetTrackingDone: Boolean = false
    private var targetTrackingLoopCounter: Int = 0
    private var targetTrackingCountdown: Int = 0
    // 0.5, 0.25, 10.0, 1.0, 8 --> 2 seconds max turning
    private val ALIGNMENT_POWER_COARSE: Double = 0.6
    private val ALIGNMENT_POWER_FINE: Double = 0.2
    private val HEADING_TOLERANCE_FINE: Double = Math.toRadians(12.0)
    private val HEADING_LIMELIGHT: Double = Math.toRadians(4.0)
    private val HEADING_TOLERANCE: Double = Math.toRadians(1.0)
    private val TRACKING_COUNTDOWN: Int = 12
    private var currentShotX: Double = 0.0
    private var currentShotY: Double = 0.0
    private var currentShotVelocity: Double = 0.0
    private var currentShotAngle: Double = 0.0

    private var gateOpen: Boolean = false
    private var intakeRunning: Boolean = false

    override fun onInit() {

        if(abs(startPose.x) < 0.1 && abs(startPose.y) < 0.1) {
            follower.setStartingPose(testingPose)
            PoseStorage.blueAlliance = true
            testMode = true
        } else {
            follower.setStartingPose(startPose)
        }

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

        Turret.setStartPos()
        val turretPos = Turret.target
        Turret.spinToPos(turretPos)

        Gate.gate_close()
    }
    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
//            FieldCentric(imu)
        )
        driverControlled.scalar = 0.9

////////////////////////////////////////////////////////////////////////////
//        GamePad 2 - Operator Commands
////////////////////////////////////////////////////////////////////////////
        //Intake artifact
        button { gamepad1.left_bumper }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                Gate.gate_close()
                Intake.spinSlowSpeed()
                intakeRunning = true
                gateOpen = false
            }
            .whenBecomesFalse {
                Intake.spinStop()
                intakeRunning = false
            }

        //Outtake artifact
        button { gamepad1.right_bumper }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                Gate.gate_close()
                Intake.spinReverse()
                intakeRunning = true
                gateOpen = false
            }
            .whenBecomesFalse {
                Intake.spinStop()
                intakeRunning = false
            }
        // Point to Target
        button { gamepad1.a }
            .whenBecomesTrue {
                targetTrackingActive = !targetTrackingActive
                targetTrackingDone = false
                targetTrackingCountdown = TRACKING_COUNTDOWN
                targetTrackingLoopCounter = 0
            }

        // Drivetrain Slow-fast speed
        button { gamepad1.y }
            .whenTrue { driverControlled.scalar = 0.4 }
            .whenFalse { driverControlled.scalar = 0.9 }

        // Reset location and heading
        button { gamepad1.b }
            .whenBecomesTrue {
                if (PoseStorage.blueAlliance) {
                    follower.setPose(Pose(136.25, 8.5, Math.toRadians(-90.0)))
                } else {
                    follower.setPose(Pose(7.75, 8.5,Math.toRadians(-90.0)))
                }
            }

////////////////////////////////////////////////////////////////////////////
//        GamePad 2 - Operator Commands
////////////////////////////////////////////////////////////////////////////

        // Fine jump turret right
        button {gamepad2.right_bumper}
            .whenBecomesTrue {
                val turretPos = Turret.target + 5.0
                Turret.spinToPos(turretPos)
            }

        // Fine jump turret left
        button {gamepad2.left_bumper}
            .whenBecomesTrue {
                val turretPos = Turret.target - 5.0
                Turret.spinToPos(turretPos)
            }

        // Coarse jump turret right
        button {gamepad2.right_trigger > 0.5 }
            .whenBecomesTrue {
                val turretPos = Turret.target + 100.0
                Turret.spinToPos(turretPos)
            }

        // Coarse jump turret left
        button {gamepad2.left_trigger > 0.5}
            .whenBecomesTrue {
                val turretPos = Turret.target - 100.0
                Turret.spinToPos(turretPos)
            }

        // Gate Open/Close
        button { gamepad2.a }
            .toggleOnBecomesTrue() //make this a button command that only opens when held // default command?
            .whenBecomesTrue {
                Gate.gate_open()
                gateOpen = true
            }
            .whenBecomesFalse {
                Gate.gate_close()
                gateOpen = false
            }

        button { gamepad2.x }
            .whenBecomesTrue {
                Gate.gate_open()
                gateOpen = true
            }

        // Start shooter and set hood angle / Stop shooter
        button { gamepad2.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                var x = follower.pose.x
                val y = follower.pose.y
                if (!PoseStorage.blueAlliance){
                    x = 144 - x  }
                val currentShot = shooterController.getShot(x, y)
                if (currentShot != null) {
                    currentShotX = currentShot.x
                    currentShotY = currentShot.y
                    currentShotVelocity = currentShot.velocity
                    currentShotAngle = currentShot.angle
                    ShooterAngle.targetPosition = currentShotAngle
                    shooterController.applyShot(currentShotVelocity, currentShotAngle)
                }
            }
            .whenBecomesFalse {
                CommandManager.scheduleCommand ( Shooter.stopShooter)
            }

        // Shoot  artifacts / stop shooting
        button { gamepad2.b }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                Gate.gate_open()
                Intake.spinSlowSpeed()
                gateOpen = true
                intakeRunning = true
            }
            .whenBecomesFalse {
                Gate.gate_close()
                Intake.spinStop()
                gateOpen = false
                intakeRunning = false
            }

        // Increase shooter velocity
        button { gamepad2.dpad_up }
            .whenBecomesTrue {
                if (currentShotVelocity < 1800 ) {
                    currentShotVelocity += 10.0
                    CommandManager.scheduleCommand(Shooter.spinAtSpeed(currentShotVelocity))
                }
            }

        // Decrease shooter velocity
        button { gamepad2.dpad_down }
            .whenBecomesTrue {
                if (currentShotVelocity > 800 ) {
                    currentShotVelocity -= 10.0
                    CommandManager.scheduleCommand(Shooter.spinAtSpeed(currentShotVelocity))
                }
            }

        // Raise shooting hood
        button { gamepad2.dpad_left }
            .whenBecomesTrue {
                if (currentShotAngle > 0.0 && currentShotAngle >= 0.519 ) {
                    currentShotAngle -= 0.02
                    ShooterAngle.targetPosition = currentShotAngle
                    CommandManager.scheduleCommand(
                        ShooterAngle.update()
                    )
                }
            }

        // Lower shooting hood
        button { gamepad2.dpad_right }
            .whenBecomesTrue {
                if (currentShotAngle > 0.0 && currentShotAngle <= 0.681 ) {
                    currentShotAngle += 0.02
                    ShooterAngle.targetPosition = currentShotAngle
                    CommandManager.scheduleCommand(
                        ShooterAngle.update()
                    )
                }
            }

        // Switch alliance (works only in test mode where Teleop was started without Auto)
        button { gamepad2.left_stick_button }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                if (testMode) {
                    PoseStorage.blueAlliance = false
                    limelight.pipelineSwitch(2)
                }
            }
            .whenBecomesFalse {
                if (testMode) {
                    PoseStorage.blueAlliance = true
                    limelight.pipelineSwitch(1)
                }
            }
    }

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
        val llResult: LLResult? = limelight.latestResult
        var llTx: Double = 99.0
        var llTy: Double = 99.0
        var llTa: Double = 99.0

        //start tracking goal
        val goal = Pose(16.0, 132.0)
        val x = abs(follower.pose.x)
        val y = abs(follower.pose.y)
        val heading = follower.heading

        var targetAngle : Double
        if (PoseStorage.blueAlliance) {
            targetAngle = Math.PI - atan2(abs(goal.y - y), abs(goal.x - x))
        }
        else {
            targetAngle = atan2(abs(goal.y - y), abs(goal.x - (144 - x)))
        }
        var headingError = targetAngle - heading
        if (headingError > Math.PI) headingError = headingError - (2 * (Math.PI))
        if (targetTrackingActive) {
            ++targetTrackingLoopCounter
            var turnPower: Double = 0.0
            if (targetTrackingLoopCounter == 1 && abs(headingError) < HEADING_TOLERANCE) {
                targetTrackingDone = true
                targetTrackingActive = false
            } else if (abs(headingError) > HEADING_TOLERANCE_FINE) {
                turnPower = if (headingError > 0) {
                    -ALIGNMENT_POWER_COARSE
                } else {
                    ALIGNMENT_POWER_COARSE
                }
            } else if (abs(headingError) > HEADING_TOLERANCE) {
                turnPower = if (headingError > 0) {
                    -ALIGNMENT_POWER_FINE
                } else {
                    ALIGNMENT_POWER_FINE
                }
            } else if (--targetTrackingCountdown > 0) {
                turnPower = if (headingError > 0) {
                    -ALIGNMENT_POWER_FINE
                } else {
                    ALIGNMENT_POWER_FINE
                }
            } else {
                targetTrackingDone = true
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

        if (llResult != null && llResult.isValid) {
            llTx = llResult.tx
            llTy = llResult.ty
            llTa = llResult.ta
        } else {
            99.0
        }
        if(PoseStorage.blueAlliance) { telemetry.addData("Alliance", "Blue") }
        else {telemetry.addData("Alliance", "Red") }
        telemetry.addData("X", "%.0f, Y: %.0f, Heading: %.1f, Target: %.1f", follower.pose.x, follower.pose.y, Math.toDegrees(follower.pose.heading), Math.toDegrees(targetAngle))
        telemetry.addData("Pointing Status", "Done: %b, Active: %b", targetTrackingDone , targetTrackingActive)
        telemetry.addData("Pointing Error", "Heading: %.1f, Limelight: (%.1f, %.1f, %.2f)", Math.toDegrees(headingError), llTx, llTy, llTa)
        telemetry.addData("Turret Pos", "Current: %.0f, Target: %.0f",  Turret.turret.currentPosition, Turret.target)
        telemetry.addData("Turret", "Ready: %b, ReadyMs: %.0f, Active: %b",  Turret.turretReady, Turret.turretReadyMs, Turret.turretActive)
        telemetry.addData("Shot","X: %.0f, Y: %.0f, Vel: %.0f, Angle: %.3f", currentShotX, currentShotY, currentShotVelocity, currentShotAngle)
        telemetry.addData("Shooter Speed", "Current: %.0f, Target: %.0f", Shooter.shooter.velocity, Shooter.target)
        telemetry.addData("Shooter", "Ready: %b, ReadyMs:  %.0f, Active: %b, Power: %.2f", Shooter.shooterReady, Shooter.shooterReadyMs, Shooter.shooterActive, Shooter.shooter.power)
        telemetry.addData("Intake Running", intakeRunning)
        telemetry.addData("Gate Open", gateOpen)

        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}