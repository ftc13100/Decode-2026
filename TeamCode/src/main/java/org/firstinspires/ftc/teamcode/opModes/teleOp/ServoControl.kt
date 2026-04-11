import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle

@TeleOp(name = "ServoControl")
class ServoControl : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                NewTurret, Lift, ShooterAngle
            ),
            BindingsComponent,
            BulkReadComponent
        )
    }

    var ptoRightPos: Double = 0.5
    var ptoLeftPos: Double = 0.5
    var shooterPos: Double = 0.0


    override fun onInit() {

    }

    override fun onStartButtonPressed() {

        NewTurret.stopTracking()
        NewTurret.toAngle(180.0)
        ShooterAngle.toPos(0.0)
        Lift.pto_lift()

//        button { gamepad1.dpad_down }
//            .whenBecomesTrue {
//                ptoLeftPos += 0.05
//                ptoRightPos -= 0.05
//                Lift.toPosLeft(ptoLeftPos)()
//                Lift.toPosRight(ptoRightPos)()
//            }
//
//        button { gamepad1.dpad_up }
//            .whenBecomesTrue {
//                ptoLeftPos -= 0.05
//                ptoRightPos += 0.05
//                Lift.toPosLeft(ptoLeftPos)()
//                Lift.toPosRight(ptoRightPos)()
//            }

        button { gamepad1.dpad_right}
            .whenBecomesTrue {
                NewTurret.incrementAngle(10.0)
            }

        button { gamepad1.dpad_left }
            .whenBecomesTrue {
                NewTurret.decrementAngle(10.0)
            }

        button {gamepad1.a}
            .whenBecomesTrue {
                shooterPos += 0.05
                ShooterAngle.toPos(shooterPos)()
            }

        button {gamepad1.b}
            .whenBecomesTrue {
                shooterPos -= 0.05
                ShooterAngle.toPos(shooterPos)()
            }

        button { gamepad1.right_bumper }
            .whenTrue {
                Lift.backRightMotor.power =  1.0
            }
            .whenBecomesFalse {
                Lift.backRightMotor.power = 0.0
            }

        button { gamepad1.left_bumper }
            .whenTrue {
                Lift.backLeftMotor.power  = -1.0
            }
            .whenBecomesFalse {
                Lift.backLeftMotor.power = 0.0
            }

//        button { gamepad1.a }
//            .whenTrue {
//                Lift.motorsOn()
//            }
//            .whenBecomesFalse {
//                Lift.motorsOff()
//            }
//
//        button { gamepad1.y }
//            .whenBecomesTrue {
//                Lift.full_Lift()
//            }

    }

    override fun onUpdate() {
        BindingManager.update()
        // Runs every loop while lift is active handles motor sync automatically
//        Lift.syncLiftMotors() //can probably have this in whenTrue and have Lift.full_Lift() in whenBecomesTrue and just hold

        telemetry.addData("current draw left",  Lift.backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))
        telemetry.addData("current draw right", Lift.backRightMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))
//        telemetry.addData("left travel ticks",  Lift.leftTravelTicks())
//        telemetry.addData("right travel ticks", Lift.rightTravelTicks())
//        telemetry.addData("tick gap (L - R)",   Lift.leftTravelTicks() - Lift.rightTravelTicks())
        telemetry.addData("servo left pos", Lift.ptoLeft.position)
        telemetry.addData("servo right pos", Lift.ptoRight.position)
        telemetry.addData("Turret Angle", "%.1f, ServoPos: %.2f", NewTurret.targetAngleRobotRef, NewTurret.targetServoPosition)
        telemetry.addData("Shooter Angle Servo Pos", ShooterAngle.servo.position)
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
        Lift.pto_drive()
    }
}