package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
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

        button { gamepad1.a }
            .whenTrue {
                Lift.motorsOn()
            }
            .whenBecomesFalse {
                Lift.motorsOff()
            }
    }

    override fun onUpdate() {
        BindingManager.update()

        telemetry.addData("current draw left", Lift.backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))
        telemetry.addData("current draw right", Lift.backRightMotor.motor.getCurrent(CurrentUnit.MILLIAMPS))

        telemetry.addData("servo left pos", ptoLeftAngle)
        telemetry.addData("servo right pos", ptoRightAngle)

        telemetry.update()

    }

    override fun onStop() {
        BindingManager.reset()
    }
}