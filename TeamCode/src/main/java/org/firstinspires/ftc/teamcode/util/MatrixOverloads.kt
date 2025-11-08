package org.firstinspires.ftc.teamcode.util

import org.ejml.simple.SimpleMatrix

operator fun SimpleMatrix.plus(other: SimpleMatrix) = this.plus(other)
operator fun SimpleMatrix.minus(other: SimpleMatrix) = this.minus(other)
operator fun SimpleMatrix.times(other: SimpleMatrix) = this.mult(other)

operator fun SimpleMatrix.plus(c: Double) = this.plus(c)
operator fun SimpleMatrix.minus(c: Double) = this.minus(c)
operator fun SimpleMatrix.times(c: Double) = this.scale(c)
