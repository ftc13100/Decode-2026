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
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

private  const val TELEMETRY_INTERVAL:Int = 250

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

    private var lastLoopTime = 0.0
    private var maxLoopTime = 0.0
    private var loopTimeAverage = 0.0
    private var lastTelemetryTime = 0.0

    var speed: Double = 0.0
    var angleShooter: Double = 0.0
    var turretAngle: Double = 0.5

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

        button { gamepad2.dpad_up }
            .whenBecomesTrue {
                speed += 100
                Shooter.spinAtSpeed(speed)()
            }

        button { gamepad2.dpad_down }
            .whenBecomesTrue {
                speed -= 100
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

        button { gamepad2.right_bumper}
            .whenBecomesTrue {
                NewTurret.incrementAngle(10.0)
            }

        button { gamepad2.left_bumper }
            .whenBecomesTrue {
                NewTurret.decrementAngle(10.0)
            }

        button { gamepad2.right_trigger > 0.5 }
            .whenTrue {
                NewTurret.incrementAngle(0.1)
            }

        button { gamepad2.left_trigger > 0.5 }
            .whenTrue {
                NewTurret.decrementAngle(0.1)
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


        button { gamepad1.x }
            .whenBecomesTrue{
                Spindexer.index0()
//                Spindexer.autoIndex(0)()
            }

        button { gamepad2.b}
            .whenBecomesTrue {
                Spindexer.autoIndex(0)()
            }

//        button { gamepad2.a }
//            .whenBecomesFalse {
//                Spindexer.index2()
//            }
//
//        button { gamepad2.y }
//            .whenBecomesFalse {
//                Spindexer.index0()
//            }
//
//        button { gamepad2.x }
//            .whenBecomesFalse {
//                Spindexer.index1()
//            }

        button {gamepad2.x}
            .toggleOnBecomesTrue()
            .whenBecomesTrue {
                NewTurret.trackTarget()
            }
            .whenBecomesFalse {
                NewTurret.stopTracking()
            }

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

        val now = System.nanoTime() / 1_000_000.0
        val telemetryTime = (now - lastTelemetryTime)
        val loopTime = (now - lastLoopTime)
        if(loopTime > maxLoopTime) maxLoopTime = loopTime
        if((loopTimeAverage < 0.5 * loopTime) || (loopTimeAverage > 1.5 * loopTime))
            loopTimeAverage = loopTime
        else
            loopTimeAverage = loopTimeAverage * 0.99 + loopTime * 0.01

        if(telemetryTime > TELEMETRY_INTERVAL )
        {
            telemetry.addData("LTavg", "%.2f, Max: %.2f", loopTimeAverage, maxLoopTime)

            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)

            telemetry.addData("newX", NewTurret.newX)
            telemetry.addData("newY", NewTurret.newY)

            telemetry.addData("heading", Math.toDegrees(follower.heading))

            telemetry.addData(
                "Intake", "%s (Power: %+1.1f, Current: %3.2f mA)",
                if (intakeRunning) {
                    "Running"
                } else {
                    "Stopped"
                }, intake.power, intake.motor.getCurrent(CurrentUnit.MILLIAMPS)
            )

            telemetry.addData("Shooter", Shooter.shooter.velocity)

            telemetry.addData("spindexer pos", Spindexer.spindexer.currentPosition)

            telemetry.addData("Full?", Spindexer.isFull)
            telemetry.addData("S0 ", Spindexer.detectColorRGB(Spindexer.color0))
            telemetry.addData("Alpha", "%.3f", Spindexer.color0.normalizedColors.alpha)
            telemetry.addData("S1 ", Spindexer.detectColorRGB(Spindexer.color1))
            telemetry.addData("Alpha", "%.3f", Spindexer.color1.normalizedColors.alpha)
            telemetry.addData("S2 ", Spindexer.detectColorRGB(Spindexer.color2))
            telemetry.addData("Alpha", "%.3f", Spindexer.color2.normalizedColors.alpha)

            telemetry.addData("Turret", "F: %.1f, R: %.1f, S: %.3f",NewTurret.targetAngleField, NewTurret.targetAngleRobotRef, NewTurret.targetServoPosition)
            telemetry.update()

            lastTelemetryTime = now
        }
        lastLoopTime = now
    }

    override fun onStop() {
        BindingManager.reset()
    }
}