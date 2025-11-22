package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.sun.tools.doclint.HtmlTag
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning
import kotlin.math.abs

@TeleOp(name = "MainTeleop")
class MainTeleop : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(),
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

    override fun onInit() {
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

        button { gamepad1.a }
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

        button { gamepad1.x }
            .whenBecomesTrue {
                CommandManager.scheduleCommand(
                    ShooterAngle.angle_up
                )
            }
            .whenBecomesFalse {
                CommandManager.scheduleCommand(
                    ShooterAngle.angle_down
                )
            }

    }

    override fun onUpdate() {
        BindingManager.update()

        driverControlled.update()

        follower.update()
        //Shooter.spinning()
        //ShooterAngle.update()

        telemetry.addData("x:", "%.2f", follower.pose.x)
        telemetry.addData("y:", "%.2f", follower.pose.y)
        telemetry.addData("heading:", "%.2f", follower.pose.heading)

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