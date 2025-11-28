package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
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
import kotlin.concurrent.schedule
import kotlin.concurrent.timer
import kotlin.math.abs
import kotlin.math.atan2

@TeleOp(name = "MainTeleop")
class MainTeleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                ShooterAngle, Shooter
            ),
            BulkReadComponent,
            BindingsComponent,
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

    private val startPose = Pose(120.0, 48.0, Math.toRadians(90.0))

    override fun onInit() {
        button { gamepad1.a}
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

       // follower.startTeleopDrive()
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

        // This drive code is for Pedro, but it seems you are using MecanumDriverControlled.
        // If follower.update() in onUpdate() correctly tracks pose with MecanumDriverControlled,
        // you can leave this commented. If your pose (x, y) doesn't update,
        // you may need to use this drive method instead of MecanumDriverControlled.

//        follower.setTeleOpDrive(
//            -gamepad1.left_stick_y.toDouble(),
//            -gamepad1.left_stick_x.toDouble(),
//            -gamepad1.right_stick_x.toDouble(),
//            true
//        )

        button { gamepad1.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { driverControlled.scalar = 0.4 }
            .whenBecomesFalse { driverControlled.scalar = 1.0 }

        button {gamepad1.right_bumper}
            .whenBecomesTrue {
                val goal = Pose(16.0, 132.0)  // 120,48
                val diff = goal - follower.pose
                val heading = follower.heading

                val targetAngle = Math.PI - atan2(abs(diff.y), abs(diff.x)) // opp over adj

                val turn : Command =
                    TurnTo(targetAngle.rad)
                turn()
                driverControlled()
            }

//pid version
//        button {gamepad1.right_bumper}
//            .whenTrue {
//                val goal = Pose(16.0, 132.0)  // 120,48
//                val diff = goal - follower.pose
//                val heading = follower.heading
//                val targetAngle = Math.PI - atan2(abs(diff.y), abs(diff.x)) // opp over adj
//
//                var turnError = targetAngle - heading
//
//                // Wrap
//                if (turnError > Math.PI) turnError = (2 * (Math.PI)) - turnError
//
//                val tolerance = Math.toRadians(2.0)
//
//                val kP = 0.5 // tune
//
//                val turnPower = if (abs(turnError) > tolerance) kP * turnError else 0.0
//
//                frontLeftMotor.power = -turnPower
//                frontRightMotor.power = turnPower
//                backLeftMotor.power = -turnPower
//                backRightMotor.power = turnPower
//
//                telemetry.addData("Turn Error", "%.2f", turnError)
//                telemetry.addData("Turn Power", "%.2f", turnPower)
//                telemetry.addData("target angle", Math.toDegrees(targetAngle))
//                telemetry.update()
//            }

//        button {gamepad1.right_bumper}
//            .whenTrue {
//                val x = abs(follower.pose.x)
//                val y = abs(follower.pose.y)
//                val heading = follower.heading
//
//                val dx = 16-x
//                val dy = 132-y
//
//                val targetAngle = 90.0 + Math.toDegrees((atan2(dy, dx)))  // opp over adj
//
//                var turnError = targetAngle - heading
//
//                // Wrap
//                if (turnError > 180) turnError -= 360
//                if (turnError < -180) turnError += 360
//
//                val tolerance = 2.0
//
//                val kP = 0.01  // tune
//
//                val turnPower = if (abs(turnError) > tolerance) kP * turnError else 0.0
//
//                frontLeftMotor.power = turnPower
//                frontRightMotor.power = -turnPower
//                backLeftMotor.power = turnPower
//                backRightMotor.power = -turnPower
//
//                telemetry.addData("Turn Error", "%.2f", turnError)
//                telemetry.addData("Turn Power", "%.2f", turnPower)
//                telemetry.update()
//            }

        button { gamepad1.a }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                val x = abs(follower.pose.x)
                val y = abs(follower.pose.y)
                val params = shooterController.getShot(x, y)
                if (params != null) {
                    shooterController.applyShot(params)
                } else {
                    telemetry.log().add("Shot not found for ($x, $y)")
                }
            }
            .whenBecomesFalse {
                Shooter.spinAtSpeed(0.0)
            }

        button { gamepad1.x }
            .whenBecomesTrue {
                intake.power = 0.7
            }
            .whenBecomesFalse {
                intake.power = 0.0
            }

        button { gamepad1.b }
            .toggleOnBecomesTrue() //make this a button command that only opens when held // default command?
            .whenBecomesTrue { Gate.gate_open }
            .whenBecomesFalse { Gate.gate_close }
    }
//        button { gamepad1.b }
//            .toggleOnBecomesTrue()
//            .whenBecomesTrue { ShooterAngle.angle_up() }
//            .whenBecomesFalse { ShooterAngle.angle_down() }
//    }

    override fun onUpdate() {
        BindingManager.update()
        driverControlled.update()
        follower.update()
        //Shooter.spinning()

        telemetry.addData("x:", "%.2f", follower.pose.x)
        telemetry.addData("y:", "%.2f", follower.pose.y)
        telemetry.addData("heading:", "%.2f", Math.toDegrees(follower.pose.heading))

        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
//            val botpose: Pose3D = result.botpose
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
        } else {
            telemetry.addData("Limelight", "Target not found")
        }

        telemetry.addData("Shooter Target Vel", Shooter.target)
        telemetry.addData("Shooter Actual Vel", "%.2f", Shooter.shooter.velocity)
        telemetry.addData("Angle Target Pos", ShooterAngle.targetPosition)
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}