package com.vio.streamer

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.vio.streamer.model.ImuSample
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.sqrt

class ImuCollector(
    context: Context,
    private val sampleQueue: ConcurrentLinkedQueue<ImuSample>
) : SensorEventListener {

    private val sensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val rotationVector =
        sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val lock = Any()
    private val readinessLock = Object()
    private val sampleHistory = ArrayList<ImuSample>(MAX_HISTORY_SAMPLES)

    @Volatile
    private var started = false

    @Volatile
    private var ready = false

    private var latestAccel: FloatArray? = null
    private var latestGyro: FloatArray? = null
    private var latestQuaternion: FloatArray? = null
    private var latestAccelTimestampNs = 0L
    private var latestGyroTimestampNs = 0L
    private var latestOrientationTimestampNs = 0L

    fun start() {
        if (started) {
            return
        }
        val accelSensor = accelerometer
            ?: throw IllegalStateException("Accelerometer not available")
        val gyroSensor = gyroscope
            ?: throw IllegalStateException("Gyroscope not available")
        val rotationSensor = rotationVector
            ?: throw IllegalStateException("Rotation vector sensor not available")
        started = true
        ready = false
        sensorManager.registerListener(this, accelSensor, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_GAME)
    }

    fun stop() {
        if (!started) {
            return
        }
        started = false
        ready = false
        sensorManager.unregisterListener(this)
        synchronized(lock) {
            latestAccel = null
            latestGyro = null
            latestQuaternion = null
            latestAccelTimestampNs = 0L
            latestGyroTimestampNs = 0L
            latestOrientationTimestampNs = 0L
            sampleHistory.clear()
        }
        sampleQueue.clear()
        synchronized(readinessLock) {
            readinessLock.notifyAll()
        }
    }

    fun awaitWarmup(timeoutMs: Long): Boolean {
        val deadlineMs = System.currentTimeMillis() + timeoutMs
        synchronized(readinessLock) {
            while (started && !ready) {
                val remainingMs = deadlineMs - System.currentTimeMillis()
                if (remainingMs <= 0L) {
                    break
                }
                readinessLock.wait(remainingMs)
            }
        }
        return ready
    }

    fun getFrameSyncedSample(frameTimestampNs: Long): ImuSample? {
        synchronized(lock) {
            if (sampleHistory.isEmpty()) {
                return null
            }
            val first = sampleHistory.first()
            if (frameTimestampNs <= first.timestampNs) {
                return first.copy(timestampNs = frameTimestampNs)
            }
            val last = sampleHistory.last()
            if (frameTimestampNs >= last.timestampNs) {
                return last.copy(timestampNs = frameTimestampNs)
            }

            for (index in sampleHistory.lastIndex downTo 0) {
                val previous = sampleHistory[index]
                if (previous.timestampNs > frameTimestampNs) {
                    continue
                }
                val next = sampleHistory.getOrNull(index + 1)
                    ?: return previous.copy(timestampNs = frameTimestampNs)
                if (next.timestampNs <= previous.timestampNs) {
                    return previous.copy(timestampNs = frameTimestampNs)
                }
                val alpha =
                    (frameTimestampNs - previous.timestampNs).toFloat() /
                        (next.timestampNs - previous.timestampNs).toFloat()
                return interpolate(previous, next, alpha, frameTimestampNs)
            }
            return last.copy(timestampNs = frameTimestampNs)
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!started) {
            return
        }
        synchronized(lock) {
            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    latestAccel = floatArrayOf(event.values[0], event.values[1], event.values[2])
                    latestAccelTimestampNs = event.timestamp
                }

                Sensor.TYPE_GYROSCOPE -> {
                    latestGyro = floatArrayOf(event.values[0], event.values[1], event.values[2])
                    latestGyroTimestampNs = event.timestamp
                }

                Sensor.TYPE_GAME_ROTATION_VECTOR,
                Sensor.TYPE_ROTATION_VECTOR -> {
                    val quaternionWxyz = FloatArray(4)
                    SensorManager.getQuaternionFromVector(quaternionWxyz, event.values)
                    latestQuaternion = floatArrayOf(
                        quaternionWxyz[1],
                        quaternionWxyz[2],
                        quaternionWxyz[3],
                        quaternionWxyz[0]
                    )
                    latestOrientationTimestampNs = event.timestamp
                }

                else -> return
            }

            val accel = latestAccel ?: return
            val gyro = latestGyro ?: return
            val quaternion = latestQuaternion ?: return
            val euler = quaternionToEuler(
                qx = quaternion[0],
                qy = quaternion[1],
                qz = quaternion[2],
                qw = quaternion[3]
            )
            val sample = ImuSample(
                timestampNs = maxOf(
                    latestAccelTimestampNs,
                    latestGyroTimestampNs,
                    latestOrientationTimestampNs
                ),
                ax = accel[0],
                ay = accel[1],
                az = accel[2],
                gx = gyro[0],
                gy = gyro[1],
                gz = gyro[2],
                qx = quaternion[0],
                qy = quaternion[1],
                qz = quaternion[2],
                qw = quaternion[3],
                pitchRad = euler[0],
                yawRad = euler[1],
                rollRad = euler[2]
            )
            sampleQueue.offer(sample)
            appendHistoryLocked(sample)
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun appendHistoryLocked(sample: ImuSample) {
        val lastSample = sampleHistory.lastOrNull()
        if (lastSample != null && sample.timestampNs < lastSample.timestampNs) {
            return
        }
        sampleHistory.add(sample)
        while (sampleHistory.size > MAX_HISTORY_SAMPLES) {
            sampleHistory.removeAt(0)
        }
        while (
            sampleHistory.size > 2 &&
            sample.timestampNs - sampleHistory.first().timestampNs > HISTORY_WINDOW_NS
        ) {
            sampleHistory.removeAt(0)
        }
        if (!ready) {
            ready = true
            synchronized(readinessLock) {
                readinessLock.notifyAll()
            }
        }
    }

    private fun interpolate(
        previous: ImuSample,
        next: ImuSample,
        alpha: Float,
        targetTimestampNs: Long
    ): ImuSample {
        val quaternion = interpolateQuaternion(previous, next, alpha)
        val euler = quaternionToEuler(
            qx = quaternion[0],
            qy = quaternion[1],
            qz = quaternion[2],
            qw = quaternion[3]
        )
        return ImuSample(
            timestampNs = targetTimestampNs,
            ax = lerp(previous.ax, next.ax, alpha),
            ay = lerp(previous.ay, next.ay, alpha),
            az = lerp(previous.az, next.az, alpha),
            gx = lerp(previous.gx, next.gx, alpha),
            gy = lerp(previous.gy, next.gy, alpha),
            gz = lerp(previous.gz, next.gz, alpha),
            qx = quaternion[0],
            qy = quaternion[1],
            qz = quaternion[2],
            qw = quaternion[3],
            pitchRad = euler[0],
            yawRad = euler[1],
            rollRad = euler[2]
        )
    }

    private fun interpolateQuaternion(
        previous: ImuSample,
        next: ImuSample,
        alpha: Float
    ): FloatArray {
        var qx2 = next.qx
        var qy2 = next.qy
        var qz2 = next.qz
        var qw2 = next.qw
        val dot =
            previous.qx * qx2 +
                previous.qy * qy2 +
                previous.qz * qz2 +
                previous.qw * qw2
        if (dot < 0f) {
            qx2 = -qx2
            qy2 = -qy2
            qz2 = -qz2
            qw2 = -qw2
        }

        var outQx = lerp(previous.qx, qx2, alpha)
        var outQy = lerp(previous.qy, qy2, alpha)
        var outQz = lerp(previous.qz, qz2, alpha)
        var outQw = lerp(previous.qw, qw2, alpha)
        val norm = sqrt(outQx * outQx + outQy * outQy + outQz * outQz + outQw * outQw)
        if (norm > NORMALIZATION_EPSILON) {
            outQx /= norm
            outQy /= norm
            outQz /= norm
            outQw /= norm
        } else {
            outQx = previous.qx
            outQy = previous.qy
            outQz = previous.qz
            outQw = previous.qw
        }
        return floatArrayOf(outQx, outQy, outQz, outQw)
    }

    private fun quaternionToEuler(
        qx: Float,
        qy: Float,
        qz: Float,
        qw: Float
    ): FloatArray {
        val sinRollCosPitch = 2f * (qw * qx + qy * qz)
        val cosRollCosPitch = 1f - 2f * (qx * qx + qy * qy)
        val roll = atan2(sinRollCosPitch, cosRollCosPitch)

        val sinPitch = 2f * (qw * qy - qz * qx)
        val pitch = if (abs(sinPitch) >= 1f) {
            if (sinPitch >= 0f) HALF_PI else -HALF_PI
        } else {
            asin(sinPitch)
        }

        val sinYawCosPitch = 2f * (qw * qz + qx * qy)
        val cosYawCosPitch = 1f - 2f * (qy * qy + qz * qz)
        val yaw = atan2(sinYawCosPitch, cosYawCosPitch)

        return floatArrayOf(pitch, yaw, roll)
    }

    private fun lerp(start: Float, end: Float, alpha: Float): Float {
        return start + (end - start) * alpha.coerceIn(0f, 1f)
    }

    private companion object {
        const val MAX_HISTORY_SAMPLES = 512
        const val HISTORY_WINDOW_NS = 5_000_000_000L
        const val NORMALIZATION_EPSILON = 1e-6f
        val HALF_PI = (Math.PI / 2.0).toFloat()
    }
}
