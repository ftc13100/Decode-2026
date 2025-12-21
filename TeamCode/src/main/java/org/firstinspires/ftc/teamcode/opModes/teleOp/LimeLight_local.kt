package org.firstinspires.ftc.teamcode.opModes.teleOp

import kotlin.math.PI
import kotlin.math.tan
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
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

    // known field positions
    private val goal = Pose(12.0, 132.0)
    private val aprilTag = Pose(12.0, 132.0)

    private val startPose = Pose(72.0, 72.0, Math.toRadians(90.0))

    // how many degrees back is your limelight rotated from perfectly vertical
    private val limelightMountAngleDegrees = 20.0

    // distance from the center of the Limelight lens to the floor
    private val limelightLensHeightInches = 12.148

    // distance from the target to the floor
    private val goalHeightInches = 29.5

    // if Limelight is angled left/right like if turret is moved
    private val limelightYawOffsetRadians = 0.0

    lateinit var limelight: Limelight3A

    private val frontLeftName = "frontLeft"
    private val frontRightName = "frontRight"
    private val backLeftName = "backLeft"
    private val backRightName = "backRight"

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

        frontLeftMotor = MotorEx(frontLeftName)
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName)
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor).forEach {
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
            Gamepads.gamepad1.rightStickX,
        )

        driverControlled.scalar = 0.9

        button { gamepad1.y }
            .whenTrue { driverControlled.scalar = 0.4 }
            .whenFalse { driverControlled.scalar = 0.95 }
    }

    // I just realized I might not be calculating this properly as in considering
    // the distance from the goal is not the exact position, need to add back in

    override fun onUpdate() {
        BindingManager.update()
        follower.update()
        driverControlled.update()

        val result = limelight.latestResult
        if (!result.isValid) {
            telemetry.addData("Limelight", "No target")
            telemetry.addData("Odo Pose", follower.pose)
            telemetry.update()
            return
        }

        // Vision distance calculation
        val targetOffsetAngleVertical = result.ty
        val angleToGoalDegrees =
            limelightMountAngleDegrees + targetOffsetAngleVertical
        val angleToGoalRadians =
            angleToGoalDegrees * (PI / 180.0)

        // calculate distance
        val distanceFromLimelightToGoalInches =
            (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians)

        // distance computed
        val r = distanceFromLimelightToGoalInches

        // Robot pose from odo
        val robotX = follower.pose.x
        val robotY = follower.pose.y
        val robotHeading = follower.pose.heading // radians

        // horizontal angle from Limelight
        val targetOffsetAngleHorizontal = 0.0 //result.tx * (PI / 180.0)

        val theta = robotHeading + limelightYawOffsetRadians + targetOffsetAngleHorizontal

        // polar to cartesian
        val dx = r * cos(theta)
        val dy = r * sin(theta)

        // estimated field position of the goal from vision
        val estimatedGoalPose = Pose(robotX + dx, robotY + dy)

        // Odo distance to goal
        val odoDistanceToGoal = sqrt((goal.x - robotX) * (goal.x - robotX) + (goal.y - robotY) * (goal.y - robotY))

        // Telemetry
        telemetry.addData("Odo Pose", follower.pose)
        telemetry.addData("Vision Distance (in)", r)
        telemetry.addData("Odo Distance to Goal (in)", odoDistanceToGoal)
        telemetry.addData("Vision Goal Pose", estimatedGoalPose)
        telemetry.addData("True Goal Pose", goal)
        telemetry.update()
    }
}
