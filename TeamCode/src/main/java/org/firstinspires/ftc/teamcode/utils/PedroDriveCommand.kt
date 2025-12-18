package org.firstinspires.ftc.teamcode.utils

import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.subsystems.Turret.turn
import java.util.function.Supplier


class PedroDriveCommand @JvmOverloads constructor(
    private val drivePower: Supplier<Double>,
    private val strafePower: Supplier<Double>,
    private val turnPower: Supplier<Double>,
    private val robotCentric: Boolean = true
) : Command() {
    override val isDone = false
    var scalar = 1.0

    override fun start() {
        follower.startTeleopDrive()
    }

    fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers
        follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
    }

    override fun update() {
        val powers = listOf(drivePower, strafePower, turnPower).map { it.get() * scalar }.toDoubleArray()
        calculateAndSetPowers(powers)
    }

    override fun stop(interrupted: Boolean) {
        ActiveOpMode.telemetry.addData("interrupted", interrupted)
        if (interrupted) {
            follower.breakFollowing()
        }
    }
}