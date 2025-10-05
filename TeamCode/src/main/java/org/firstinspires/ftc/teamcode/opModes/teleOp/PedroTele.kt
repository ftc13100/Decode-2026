package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.HolonomicMode
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower

@TeleOp(name = "PedroTele")
class PedroTele : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(),
            BulkReadComponent,
            BindingsComponent,
            PedroComponent(Constants::createFollower)
        )
    }

        // change the names and directions to suit your robot
        private val frontLeftMotor = MotorEx("front_left").reversed()
        private val frontRightMotor = MotorEx("front_right")
        private val backLeftMotor = MotorEx("back_left").reversed()
        private val backRightMotor = MotorEx("back_right")
        private val imu = IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()

    override fun onStartButtonPressed() {
            val driverControlled = MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1.leftStickY,
                Gamepads.gamepad1.leftStickX,
                Gamepads.gamepad1.rightStickX
            )
            driverControlled()

            Gamepads.gamepad2.dpadUp whenBecomesTrue Lift.toHigh whenBecomesFalse Claw.open

            (Gamepads.gamepad2.rightTrigger greaterThan 0.2)
                .whenBecomesTrue(
                    Claw.close.then(Lift.toHigh)
                )

            Gamepads.gamepad2.leftBumper whenBecomesTrue Claw.open.and(Lift.toLow)
        }
    }