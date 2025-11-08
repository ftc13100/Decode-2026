package org.firstinspires.ftc.teamcode.util

import org.ejml.simple.SimpleMatrix

class EKF(
    var x: SimpleMatrix, // initial state vector (n x 1)
    var P: SimpleMatrix, // initial covariance (n x n)
    val Q: SimpleMatrix, // process noise covariance (n x n)
    val R: SimpleMatrix  // measurement noise covariance (m x m)
) {
    // Nonlinear state transition function f(x, u): SimpleMatrix(n x 1), u: SimpleMatrix
    lateinit var f: (SimpleMatrix, SimpleMatrix) -> SimpleMatrix

    // State transition Jacobian F(x, u): SimpleMatrix(n x n)
    lateinit var fJacobian: (SimpleMatrix, SimpleMatrix) -> SimpleMatrix

    // Measurement function h(x): SimpleMatrix(m x 1)
    lateinit var h: (SimpleMatrix) -> SimpleMatrix

    // Measurement Jacobian H(x): SimpleMatrix(m x n)
    lateinit var hJacobian: (SimpleMatrix) -> SimpleMatrix

    fun predict(u: SimpleMatrix) {
        // Predict state
        x = f(x, u)
        // Predict covariance
        val F = fJacobian(x, u)
        P = F * P * F.transpose() + Q
    }

    fun update(z: SimpleMatrix) {
        val H = hJacobian(x)
        val y = z -h(x) // innovation
        val S = H * P * H.transpose() + R
        val K = P * H.transpose() * S.invert() // Kalman gain
        x = x + K * y // update state
        val I = SimpleMatrix.identity(x.numRows)
        P = (I - K * H) * P // update covariance
    }
}
