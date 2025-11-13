package com.hamosad.lib.math

data class PIDGains(val p: Double = 0.0, val i: Double = 0.0, val d: Double = 0.0, val f: Double = 0.0)

class PIDController(private var p: Double = 0.0, private var i: Double = 0.0, private var d: Double = 0.0, private var f: Double = 0.0) {
    constructor(gains: PIDGains): this(gains.p, gains.i, gains.d, gains.f)

    fun updateGains(newP: Double = p, newI: Double = i, newD: Double = d, newFeedForward: Double = f) {
        p = newP
        i = newI
        d = newD
        f = newFeedForward
    }

    private var lastTimestamp: Long = 0
    private var lastIntegral: Double = 0.0
    private var lastError: Double = 0.0
    var setpoint: Double = 0.0
    fun calculate(measurement: Double, newSetpoint: Double = setpoint): Double {
        val deltaT: Long = if (lastTimestamp != 0L) System.currentTimeMillis() - lastTimestamp else 0L
        lastTimestamp = System.currentTimeMillis()

        setpoint = newSetpoint
        val error = setpoint - measurement
        val proportional = p * error
        val integral = lastIntegral + i * (error * deltaT)
        val derivative = (error - lastError) / deltaT

        lastError = error
        lastIntegral = integral

        return proportional + integral + derivative + f
    }

    fun integralReset() {
        lastIntegral = 0.0
    }

    fun resetController() {
        lastTimestamp = 0L
        lastError = 0.0
        integralReset()
    }
}