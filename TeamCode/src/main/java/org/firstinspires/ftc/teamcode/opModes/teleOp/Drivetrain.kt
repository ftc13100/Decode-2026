package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
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
                Intake, Spindexer, Shooter, ShooterAngle
            ),
            BindingsComponent,
            BulkReadComponent
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

    override fun onInit() {

        frontLeftMotor = MotorEx(frontLeftName).reversed()
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName).reversed()
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

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

        //Intake artifact
        button { gamepad1.left_bumper }
            .whenTrue {
                Intake.spinFast()
            }
            .whenBecomesFalse {
                Intake.spinStop()
            }

        //Outtake artifact
        button { gamepad1.right_bumper }
            .whenTrue {
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
            }

        button { gamepad1.a }
            .whenTrue {
                Spindexer.spinIndex()
            }
            .whenBecomesFalse {
                Spindexer.stopShot()
            }

        button { gamepad1.dpad_up }
            .whenBecomesTrue {
                Shooter.shootSpeed(0.6)
            }
            .whenBecomesFalse {
                Shooter.shootSpeed(0.0)
            }

    }

    override fun onUpdate() {
        BindingManager.update()

//        if (!Lift.isRunning) {
//            driverControlled.update()
//        }

        // loop time check
//        val now = System.nanoTime()
//        if (lastLoopTime != 0L) {
//            loopTimeMs = (now - lastLoopTime) / 1_000_000.0
//        }
//        lastLoopTime = now

        // telemetery
//        telemetry.addData("Loop Time (ms)", "%.2f", loopTimeMs)


        telemetry.addData(
            "Intake", "%s (Power: %+1.1f, Current: %3.2f mA)",
            if (intakeRunning) {
                "Running"
            } else {
                "Stopped"
            }, intake.power, intake.motor.getCurrent(CurrentUnit.MILLIAMPS)
        )

        telemetry.addData("Full?", Spindexer.isFull)
        telemetry.addData("S0 ", Spindexer.detectColorRGB(Spindexer.color0))
        telemetry.addData("Alpha", "%.3f", Spindexer.color0.normalizedColors.alpha)
        telemetry.addData("S1 ", Spindexer.detectColorRGB(Spindexer.color1))
        telemetry.addData("Alpha", "%.3f", Spindexer.color1.normalizedColors.alpha)
        telemetry.addData("S2 ", Spindexer.detectColorRGB(Spindexer.color2))

        telemetry.update()

    }

    override fun onStop() {
        BindingManager.reset()
    }
}