package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intakeRunning
import org.firstinspires.ftc.teamcode.opModes.subsystems.Lift
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@TeleOp(name = "Drivetrain")
class Drivetrain : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Intake, Spindexer, Shooter, ShooterAngle, NewTurret
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


    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    private var lastLoopTime = 0L
    private var loopTimeMs = 0.0

    var speed: Double = 0.0
    var angleShooter: Double = 0.0
    var turretAngle: Double = 0.0

    private val testingPose = Pose(72.0, 72.0, Math.toRadians(90.0))

    override fun onInit() {

        follower.setStartingPose(testingPose)

        ShooterAngle.angle_mid()

        frontLeftMotor = MotorEx(frontLeftName)
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName)
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

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
            Gamepads.gamepad1.rightStickX,
        )
        driverControlled.scalar = 0.95

        // slow mode
        button { gamepad1.y }
            .whenTrue { driverControlled.scalar = 0.4 }
            .whenFalse { driverControlled.scalar = 0.95 }

        // lift
//        button { gamepad1.dpad_up}
//            .whenBecomesTrue { Lift.full_Lift }

        button { gamepad1.dpad_up }
            .whenBecomesTrue {
                speed += 10
                Shooter.spinAtSpeed(speed)()
            }

        button { gamepad1.dpad_down }
            .whenBecomesTrue {
                speed -= 10
                Shooter.spinAtSpeed(speed)()
            }

        button { gamepad1.dpad_right }
            .whenBecomesTrue {
                angleShooter += 0.05
                ShooterAngle.toAngle(angleShooter)()
            }

        button { gamepad1.dpad_left }
            .whenBecomesTrue {
                angleShooter -= 0.05
                ShooterAngle.toAngle(angleShooter)()
            }

        button { gamepad2.dpad_right }
            .whenBecomesTrue {
                turretAngle += 0.02
                NewTurret.toAngle(turretAngle)()
            }

        button { gamepad2.dpad_left }
            .whenBecomesTrue {
                turretAngle -= 0.02
                NewTurret.toAngle(turretAngle)()
            }

        //Intake artifact
        button { gamepad1.left_bumper }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                Intake.spinFast()
            }
            .whenBecomesFalse {
                Intake.spinStop()
            }

        //Outtake artifact
        button { gamepad1.right_bumper }
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                Intake.spinReverse()
            }
            .whenBecomesFalse {
                Intake.spinStop()
            }

        button { gamepad1.b }
            .whenTrue {
                Spindexer.spinShot()
            }
            .whenBecomesFalse {
                Spindexer.stopShot()
                Intake.spinStop()
            }

        button { gamepad1.a }
            .whenTrue {
                Spindexer.spinIndex()
            }
            .whenBecomesFalse {
                Spindexer.stopShot()
                Intake.spinStop()
            }

        button { gamepad1.x }
            .whenBecomesTrue{
                Spindexer.index0()
//                Spindexer.autoIndex(0)
            }

//        button { gamepad1.dpad_up }
//            .toggleOnBecomesTrue()
//            .whenBecomesTrue {
//                speed += 10
//                Shooter.spinAtSpeed(speed).schedule()
//            }

    }

    override fun onUpdate() {
        BindingManager.update()
        driverControlled.update()
//        NewTurret.toMid()
//        ShooterAngle.angle_mid()
//        Shooter.stallShooter()
        follower.update()

//        if (!Lift.isRunning) {
//            driverControlled.update()
//        }

        // loop time check
        val now = System.nanoTime()
        if (lastLoopTime != 0L) {
            loopTimeMs = (now - lastLoopTime) / 1_000_000.0
        }
        lastLoopTime = now

//      telemetry
        telemetry.addData("Loop Time (ms)", "%.2f", loopTimeMs)

        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)

        telemetry.addData(
            "Intake", "%s (Power: %+1.1f, Current: %3.2f mA)",
            if (intakeRunning) {
                "Running"
            } else {
                "Stopped"
            }, intake.power, intake.motor.getCurrent(CurrentUnit.MILLIAMPS)
        )

        telemetry.addData("Spindexer", Spindexer.spindexer.motor.getCurrent(CurrentUnit.MILLIAMPS))

        telemetry.addData("Shooter", Shooter.shooter.velocity)

        telemetry.addData("spindexer", Spindexer.spindexer.currentPosition)

        telemetry.addData("Full?", Spindexer.isFull)
        telemetry.addData("S0 ", Spindexer.detectColorRGB(Spindexer.color0))
        telemetry.addData("Alpha", "%.3f", Spindexer.color0.normalizedColors.alpha)
        telemetry.addData("S1 ", Spindexer.detectColorRGB(Spindexer.color1))
        telemetry.addData("Alpha", "%.3f", Spindexer.color1.normalizedColors.alpha)
        telemetry.addData("S2 ", Spindexer.detectColorRGB(Spindexer.color2))
        telemetry.addData("Alpha", "%.3f", Spindexer.color2.normalizedColors.alpha)


        telemetry.update()

    }

    override fun onStop() {
        BindingManager.reset()
    }
}