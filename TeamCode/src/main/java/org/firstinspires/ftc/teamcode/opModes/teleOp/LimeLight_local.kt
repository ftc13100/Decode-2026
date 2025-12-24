package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.pedropathing.geometry.Pose
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

@Configurable
@TeleOp(name = "lime@local")
class LimeLightLocal : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(),
            BindingsComponent,
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    companion object {
        @JvmField
        var limelightMountAngleDegrees = 0.0
    }

    private val panelsTelemetry = PanelsTelemetry.telemetry

    // Known field position
    private val goal = Pose(16.0, 132.0)

    private val startPose = Pose(72.0, 72.0, Math.toRadians(90.0))

    // Limelight geometry
    private val limelightLensHeightInches = 12.148
    private val goalHeightInches = 29.5

    // Vision results
    private var visionDistance = 0.0
    private var estimatedGoalPose: Pose? = null

    lateinit var limelight: Limelight3A

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    override fun onInit() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 25

        limelight.pipelineSwitch(1)
        limelight.start()

        frontLeftMotor = MotorEx("frontLeft")
        frontRightMotor = MotorEx("frontRight")
        backLeftMotor = MotorEx("backLeft")
        backRightMotor = MotorEx("backRight")

        listOf(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        follower.setStartingPose(startPose)
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

        driverControlled.scalar = 0.9

        button { gamepad1.y }
            .whenTrue { driverControlled.scalar = 0.4 }
            .whenFalse { driverControlled.scalar = 0.95 }
    }

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
        driverControlled.update()

        val robotPose = follower.pose
        val result = limelight.latestResult

        if (result.isValid) {
            // Vertical angle for distance
            val angleToGoalDegrees = limelightMountAngleDegrees + result.ty
            val angleToGoalRadians = angleToGoalDegrees * (PI / 180.0)

            visionDistance = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)

            // Horizontal offset
            val horizontalRadians = result.tx * (PI / 180.0)
            val theta = robotPose.heading + horizontalRadians

            val dx = visionDistance * cos(theta)
            val dy = visionDistance * sin(theta)

            estimatedGoalPose = Pose(
                robotPose.x + dx,
                robotPose.y + dy
            )
        } else {
            estimatedGoalPose = null
        }

        // Odometry-only distance
        val odoDistanceToGoal = sqrt((goal.x - robotPose.x).pow(2.0) + (goal.y - robotPose.y).pow(2.0))

        // ================= TELEMETRY =================

        telemetry.addLine("=== Vision ===")
        if (result.isValid && estimatedGoalPose != null) {
            telemetry.addData("Vision Distance (in)", visionDistance)
            telemetry.addData("tx (deg)", result.tx)
            telemetry.addData("ty (deg)", result.ty)
            telemetry.addData("Vision Goal Pose", estimatedGoalPose)
        } else {
            telemetry.addData("Limelight", "Target Not Found")
        }

        telemetry.addLine("=== Odometry ===")
        telemetry.addData("Robot Pose", robotPose)
        telemetry.addData("Odo Distance to Goal", odoDistanceToGoal)

        telemetry.addLine("=== Debug ===")
        telemetry.addData("Mount Angle (deg)", limelightMountAngleDegrees)

        panelsTelemetry.update(telemetry)
        telemetry.update()
    }
}
