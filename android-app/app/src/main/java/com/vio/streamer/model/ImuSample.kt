package com.vio.streamer.model

data class ImuSample(
    val timestampNs: Long,
    val ax: Float,
    val ay: Float,
    val az: Float,
    val gx: Float,
    val gy: Float,
    val gz: Float,
    val qx: Float,
    val qy: Float,
    val qz: Float,
    val qw: Float,
    val pitchRad: Float,
    val yawRad: Float,
    val rollRad: Float
)
