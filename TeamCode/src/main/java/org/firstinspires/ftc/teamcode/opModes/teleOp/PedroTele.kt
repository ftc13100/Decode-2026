package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
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
        private val frontLeftMotor = MotorEx("leftFront").reversed()
        private val frontRightMotor = MotorEx("rightFront")
        private val backLeftMotor = MotorEx("leftRear").reversed()
        private val backRightMotor = MotorEx("rightRear")

    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
        )
        driverControlled()

    }
    }