package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.blueLime.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.redLime
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.turret

@TeleOp(name = "Tele")
class Tele : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(turret, Shooter, blueLime),
            BulkReadComponent,
            BindingsComponent
        )
    }
    // change the names and directions to suit your robot
    private val frontLeftMotor = MotorEx("frontLeft").reversed()
    private val frontRightMotor = MotorEx("frontRight")
    private val backLeftMotor = MotorEx("backLeft").reversed()
    private val backRightMotor = MotorEx("backRight")


    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled()

    }

    override fun onUpdate() {
        val result: LLResult? = limelight.latestResult
        if (result == null) {
            telemetry.addData("Limelight", "Target not found")
        }
        if (result != null && result.isValid) {
            telemetry.addData("tx (Horizontal Error)", "%.2f", result.tx)
            telemetry.addData("ty (Vertical Error)", "%.2f", result.ty)
            telemetry.update()
        }

        if (result!= null && result.isValid && result.tx > 3) {
            turret.spinRight()
            } else if (result!= null && result.isValid && result.tx < -3) {
                turret.spinLeft()
            } else if (result == null) {
                turret.stop()
                telemetry.addData("not tracking",null)
                telemetry.update()
            }
        }
    }
