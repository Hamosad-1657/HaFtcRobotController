package com.hamosad.lib.math

import kotlin.math.abs

fun continuousDeadband(value: Double, deadband: Double): Double {
    if (deadband !in 0.0..1.0) {
        return value
        //TODO: ADD TELEMETRY NOT MY PROBLEM GOOD LUCK P.S LOVE AMIT
    }
    if (value !in -1.0..1.0) {
        return value
    }

    return if (value > deadband) {
        mapRange(value, deadband, 1.0, 0.0, 1.0)
    } else if (value < -deadband) {
        mapRange(value, -1.0, -deadband, -1.0, 0.0)
    } else {
        0.0
    }
}

fun mapRange(value: Double, startMin: Double, startMax: Double, endMin: Double, endMax: Double): Double {
    if (startMin >= startMax) {
        return value
    }
    if (endMin >= endMax) {
        return value
    }
    return endMin + (endMax - endMin) / (startMax - startMin) * (value - startMin)
}

fun simpleDeadband(value: Double, deadband: Double): Double {
    if (deadband < 0.0) {
        return value
    }
    return if (abs(value) >= deadband) value else 0.0
}
