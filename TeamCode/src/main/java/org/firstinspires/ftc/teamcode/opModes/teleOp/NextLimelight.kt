package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

@TeleOp(name = "TeleOp with Limelight")
class TeleOpWithLimelight : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(),
            BindingsComponent,
            BulkReadComponent,
        )
    }

    // Motor names
    private val frontLeftName = "leftFront"
    private val frontRightName = "rightFront"
    private val backLeftName = "leftRear"
    private val backRightName = "rightRear"

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    // Limelight
    private lateinit var limelight: Limelight3A

    override fun onInit() {
        // Motors
        frontLeftMotor = MotorEx(frontLeftName).reversed()
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName).reversed()
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // Limelight init
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 11
        limelight.pipelineSwitch(1)
        limelight.start()
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled.scalar = 1.0

        button { gamepad1.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { driverControlled.scalar = 0.4 }
            .whenBecomesFalse { driverControlled.scalar = 1.0 }
    }

    override fun onUpdate() {
        BindingManager.update()
        driverControlled.update()

        // Limelight data
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val botpose: Pose3D = result.botpose
            telemetry.addData("tx", result.tx)
            telemetry.addData("ty", result.ty)
            telemetry.addData("Botpose", botpose.toString())
        }

        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}
