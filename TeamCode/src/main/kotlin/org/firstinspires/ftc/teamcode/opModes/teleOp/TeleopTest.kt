package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.Component
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower

@TeleOp(name = "NextFTC Main TeleOp")
class TeleopTest: NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(),
            BindingsComponent,
            BulkReadComponent,
            PedroComponent(Constants::createFollower)


        )
    }

    // Change the motor names to suit your robot.
    val frontLeftName = "leftFront"
    val frontRightName = "rightFront"
    val backLeftName = "leftRear"
    val backRightName = "rightRear"

    val frontLeftMotor = MotorEx(frontLeftName)
    val frontRightMotor = MotorEx(frontRightName)
    val backLeftMotor = MotorEx(backLeftName)
    val backRightMotor = MotorEx(backRightName)

    lateinit var driverControlled: MecanumDriverControlled


    override fun onInit() {
        //braking instead of coasting
        frontLeftMotor.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftMotor.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRightMotor.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightMotor.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            backLeftMotor,
            frontRightMotor,
            backRightMotor,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled.scalar = 1.0
        //put subsystems here
    }

    override fun onUpdate() {
        BindingManager.update()
        //this.telemetry.addData("Position", Arm.armMotor.currentPosition)
        driverControlled()
        this.telemetry.update()
        driverControlled()
    }

    override fun onStop() {
        BindingManager.reset()

    }
}