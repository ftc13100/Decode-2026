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
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
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
import org.firstinspires.ftc.teamcode.opModes.subsystems.GoalFinder
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.opencv.core.Mat
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.seconds
@TeleOp(name = "MainTeleop")
class MainTeleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                ShooterAngle, Shooter, Gate, Intake, Turret, PoseStorage, GoalFinder
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

    lateinit var limelight: Limelight3A
    private val startPose = PoseStorage.poseEnd  //(72.0,72.0, Math.toRadians(90.0))
    private val testingPose = Pose(72.0,72.0,Math.toRadians(90.0))
    private var testMode: Boolean = false
    private var currentShotDistance: Double = 0.0
    private var currentShotVelocity: Double = 0.0
    private var currentShotAngle: Double = 0.0
    private var gateOpen: Boolean = false
    private var intakeRunning: Boolean = false
    private var debugTelemetry = false
    private var initialized = false;

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

        //Gate.gate_close()
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
        driverControlled.scalar = 0.95

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
                if(! GoalFinder.gfActive) {
                    GoalFinder.findGoal()
                } else {
                    GoalFinder.stop()
                }
            }

        // Turret Tracking
        button { gamepad1.x }
            .whenBecomesTrue {
                Turret.trackTarget()
                }

        // Drivetrain Slow-fast speed
        button { gamepad1.y }
            .whenTrue { driverControlled.scalar = 0.4 }
            .whenFalse { driverControlled.scalar = 0.95 }

        // Reset location and heading
        Gamepads.gamepad1.leftTrigger.asButton { it > 0.5 } and Gamepads.gamepad1.rightTrigger.asButton { it > 0.5 }
            .whenBecomesTrue {
                if (PoseStorage.blueAlliance) {
                    follower.setPose(Pose(135.75, 8.5, Math.toRadians(-90.0)))
                } else {
                    follower.setPose(Pose(8.25, 8.5,Math.toRadians(-90.0)))
                }
            }

////////////////////////////////////////////////////////////////////////////
//        GamePad 2 - Operator Commands
////////////////////////////////////////////////////////////////////////////

        // Fine jump turret right
        button {gamepad2.right_bumper}
            .whenBecomesTrue {
                Turret.turn(50.0)
            }

        // Fine jump turret left
        button {gamepad2.left_bumper}
            .whenBecomesTrue {
                Turret.turn(-50.0)
            }

        // Coarse jump turret right
        button {gamepad2.right_trigger > 0.5 }
            .whenBecomesTrue {
                Turret.turn(500.0)
            }

        // Coarse jump turret left
        button {gamepad2.left_trigger > 0.5}
            .whenBecomesTrue {
                Turret.turn(-500.0)
            }

        // turret tracking goal
        button { gamepad2.a }
            .whenBecomesTrue {
                GoalFinder.adjustToLL()
            }

        button { gamepad2.x }
            .whenBecomesTrue {
                debugTelemetry = ! debugTelemetry
                Gate.gate_open()
                gateOpen = true
            }
// Start shooter and set hood angle / Stop shooter
        button { gamepad2.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                val pose = follower.pose
                var adjX = pose.x
                val y = pose.y
                if (!PoseStorage.blueAlliance) {
                    adjX = 144.0 - adjX
                }
                // Calculate 3D Distance to Goal
                val goalDistance = sqrt((adjX - goal.x).pow(2.0) + (y - goal.y).pow(2.0) + shooterToGoalZSqrd)
                val currentShot = shooterController.getShot(goalDistance)

                val commands = SequentialGroup(
                    WaitUntil{ true },
                    InstantCommand {
                        currentShotVelocity = currentShot!!.velocity
                        currentShotAngle = currentShot.angle
                        currentShotDistance = currentShot.distance
                        shooterController.applyShot(currentShot)
                    },
                    WaitUntil { Shooter.shooterReady && abs(Turret.target - Turret.turret.currentPosition) < 3.0 },
                    InstantCommand {
                        Gate.gate_open()
                        Intake.spinSlowSpeed()
                        gateOpen = true
                        intakeRunning = true
                    }
                )
                commands()
            }
            .whenBecomesFalse {
                CommandManager.scheduleCommand(Shooter.stopShooter)
                Gate.gate_close()
                Intake.spinStop()
                gateOpen = false
                intakeRunning = false
            }

        // Shoot  artifacts / stop shooting
//        button { gamepad2.b }
//            .toggleOnBecomesTrue()
//            .whenBecomesTrue {
//                if ( {
//
//                }
//            }
//            .whenBecomesFalse {
//
//            }

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

        // lower shooting hood
        button { gamepad2.dpad_left }
            .whenBecomesTrue {
                if (currentShotAngle > 0.0 && currentShotAngle <= 0.681 ) {
                    currentShotAngle += 0.02
                    ShooterAngle.targetPosition = currentShotAngle
                    CommandManager.scheduleCommand(
                        ShooterAngle.update()
                    )
                }
            }

        // raise shooting hood
        button { gamepad2.dpad_right }
            .whenBecomesTrue {
                if (currentShotAngle > 0.0 && currentShotAngle >= 0.519 ) {
                    currentShotAngle -= 0.02
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

        button { gamepad2.right_stick_button }
            .whenBecomesTrue {
                Turret.resetToStartPosition()
            }

//        button { gamepad2.back }
//            .whenBecomesTrue {
//                debugTelemetry = ! debugTelemetry
//            }
    }

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
        driverControlled.update()

        if(!initialized)
        {
            Turret.initPos()
            initialized = true
        }

        val llResult: LLResult? = limelight.latestResult
        val turnPower = GoalFinder.calculate(follower.pose, follower.heading, llResult, PoseStorage.blueAlliance)

        if(GoalFinder.gfActive)
        {
            frontLeftMotor.power = turnPower
            frontRightMotor.power = -turnPower
            backLeftMotor.power = turnPower
            backRightMotor.power = -turnPower
        } else {
            // Manual Control
            driverControlled.update()
        } // end tracking goal

        if(PoseStorage.blueAlliance) { telemetry.addData("Alliance", "BLUE") }
        else {telemetry.addData("Alliance", "RED") }

        if(debugTelemetry) {
            telemetry.addData(
                "X",
                "%.1f, Y: %.1f, Heading: %.1f, Goal: %.1f",
                follower.pose.x,follower.pose.y,Math.toDegrees(follower.heading),Math.toDegrees(GoalFinder.gfTargetAngle)
            )
        } else {
            telemetry.addData("X",
                "%.1f, Y: %.1f, Heading: %.1f",
                follower.pose.x, follower.pose.y, Math.toDegrees(follower.heading))

            telemetry.addData("Distance", "%.2f", GoalFinder.gfGoalDistance)
        }

        if(debugTelemetry) {
            telemetry.addData(
                "Pointing",
                "Active: %b, Done: %b, DoneMs: %.0f",
                GoalFinder.gfActive,GoalFinder.gfDone, GoalFinder.gfDoneMs)
            telemetry.addData(
                "PointingVals",
                "Error: %.1f, Limelight: (%.1f, %.1f, %.2f), Dist: %.2f",
                Math.toDegrees(GoalFinder.gfHeadingError),GoalFinder.gfLLTx,GoalFinder.gfLLTy,GoalFinder.gfLLTa, GoalFinder.gfGoalDistance)
            telemetry.addData(
                "PointingDbg",
                "AnglesValid: %b, LLValid: %b, GoalAprilTagAdj: %.2f",
                GoalFinder.gfAnglesValid,GoalFinder.gfLLValid, Math.toDegrees(GoalFinder.gfGoalAprilTagAdj))

            telemetry.addData("PointingDbg2",
                "TurretAdjLL: %.0f, TurretAdjGoalAT: %.0f, Total: %.0f",
                GoalFinder.gfTurretAdjLL,GoalFinder.gfTurretAdjGoalAprilTag, GoalFinder.gfTurretAdj)

            telemetry.addData("Turret",
                "Active: %b, Ready: %b, ReadyMs: %.0f",
                Turret.turretActive, Turret.turretReady, Turret.turretReadyMs)

            telemetry.addData("TurretPos",
                "Current: %.2f, Target: %.2f, Ticks: %.2f, tolCount: %d",
                (Turret.turret.currentPosition/34.14302083),(Turret.target/34.14302083), Turret.turret.currentPosition, Turret.turretTolearanceCount)
        } else {
            telemetry.addData("Pointing", "Error: %+3.1f Limelight: +%2.1f",
                Math.toDegrees(GoalFinder.gfHeadingError), GoalFinder.gfLLTx)
        }
        if(debugTelemetry) {
            telemetry.addData("Shot Lookup", "Distance: %.2f", currentShotDistance)
        }
        telemetry.addData(
            "Shooter",
            "Target: %4.0f, Current: %4.0f, ShotDistance: %.2f, Ready: %s,",
            Shooter.target, Shooter.shooter.velocity, currentShotDistance, if(Shooter.shooterReady) { "Yes" } else { "No" })
        if(debugTelemetry) {
            telemetry.addData("Shooter",
                "ReadyMs: %.0f, Active: %b, Power: %.2f",
                Shooter.shooterReadyMs, Shooter.shooterActive, Shooter.shooter.power)
        }
        telemetry.addData("Shooter Angle:","%.3f", currentShotAngle)

        telemetry.addData("Gate",
            "%s",
            if(gateOpen) { "Open" } else { "Closed" })

        telemetry.addData("Intake",
            "%s (Power: %+1.1f)",
            if (intakeRunning) { "Running" } else { "Stopped" }, Intake.intake.power)

        telemetry.update()
    }
    override fun onStop() {
        BindingManager.reset()
    }
}