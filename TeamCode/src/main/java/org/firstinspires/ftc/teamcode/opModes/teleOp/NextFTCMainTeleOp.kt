package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx

@TeleOp(name = "NextFTC Main TeleOp")
class NextFTCMainTeleOp : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(),
            BindingsComponent,
            BulkReadComponent,
        )
    }

    // Motor names (change if needed)
    private val frontLeftName = "leftFront"
    private val frontRightName = "rightFront"
    private val backLeftName = "leftRear"
    private val backRightName = "rightRear"

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    override fun onInit() {
        // Initialize motors AFTER hardwareMap is ready
        frontLeftMotor = MotorEx(frontLeftName).reversed()
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName).reversed()
        backRightMotor = MotorEx(backRightName)

        // Braking instead of coasting
        listOf(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
           - Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled.scalar = 1.0
        // Put other subsystems here


        button { gamepad1.y}
            .toggleOnBecomesTrue()
            .whenBecomesTrue { driverControlled.scalar = 0.4 } // runs every other rising edge, including the first one
            .whenBecomesFalse { driverControlled.scalar = 1.0 } // runs the rest of the rising edges
    }

    override fun onUpdate() {
        BindingManager.update()

        driverControlled.update() // safer than driverControlled()

        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}