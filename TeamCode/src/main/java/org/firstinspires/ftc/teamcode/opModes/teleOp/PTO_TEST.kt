import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opModes.subsystems.Lift
import org.firstinspires.ftc.teamcode.opModes.subsystems.Lift.backLeftMotor
import org.firstinspires.ftc.teamcode.opModes.subsystems.Lift.backRightMotor

@TeleOp(name = "PTO_TEST")
class PTO_TEST : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Lift
            ),
            BindingsComponent,
            BulkReadComponent
        )
    }

    var ptoRightAngle: Double = 0.5
    var ptoLeftAngle: Double = 0.5


    override fun onInit() {
        Lift.pto_drive()
    }

    override fun onStartButtonPressed() {

        button { gamepad1.dpad_down }
            .whenBecomesTrue {
                Lift.pto_lift()
            }

        button { gamepad1.dpad_up }
            .whenBecomesTrue {
                Lift.pto_drive()
            }

        button { gamepad1.right_bumper }
            .whenTrue {
                backRightMotor.power =  1.0
            }
            .whenBecomesFalse {
                backRightMotor.power = 0.0
            }

        button { gamepad1.left_bumper }
            .whenTrue {
                backLeftMotor.power  = -1.0
            }
            .whenBecomesFalse {
                backLeftMotor.power = 0.0
            }

        button { gamepad1.a }
            .whenTrue {
                Lift.motorsOn()
            }
            .whenBecomesFalse {
                Lift.motorsOff()
            }

        button { gamepad1.y }
            .whenBecomesTrue {
                Lift.full_Lift()
            }

    }

    override fun onUpdate() {
        BindingManager.update()
        // Runs every loop while lift is active handles motor sync automatically
        Lift.syncLiftMotors() //can probably have this in whenTrue and have Lift.full_Lift() in whenBecomesTrue and just hold

        telemetry.addData("current draw left",  Lift.backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))
        telemetry.addData("current draw right", Lift.backRightMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))
        telemetry.addData("left travel ticks",  Lift.leftTravelTicks())
        telemetry.addData("right travel ticks", Lift.rightTravelTicks())
        telemetry.addData("tick gap (L - R)",   Lift.leftTravelTicks() - Lift.rightTravelTicks())
        telemetry.addData("servo left pos", ptoLeftAngle)
        telemetry.addData("servo right pos", ptoRightAngle)
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}