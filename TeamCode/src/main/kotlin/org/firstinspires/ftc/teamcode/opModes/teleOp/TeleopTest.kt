package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx

@TeleOp(name = "NextFTC Main TeleOp")
class TeleopTest: NextFTCOpMode() {

    // Change the motor names to suit your robot.
    val frontLeftName = "leftFront"
    val frontRightName = "rightFront"
    val backLeftName = "leftRear"
    val backRightName = "rightRear"

    val frontLeftMotor = MotorEx(frontLeftName)
    val frontRightMotor = MotorEx(frontRightName)
    val backLeftMotor = MotorEx(backLeftName)
    val backRightMotor = MotorEx(backRightName)

    lateinit var motors: Array<MotorEx>

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
        driverControlled()

        //put subsystems here
    }

    override fun onUpdate() {
        //this.telemetry.addData("Position", Arm.armMotor.currentPosition)
        this.telemetry.update()
    }
}