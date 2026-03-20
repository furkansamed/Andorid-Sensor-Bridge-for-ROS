package com.vio.streamer

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.vio.streamer.model.ImuSample
import java.util.concurrent.ConcurrentLinkedQueue

class ImuCollector(
    context: Context,
    private val sampleQueue: ConcurrentLinkedQueue<ImuSample>
) : SensorEventListener {

    private val sensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val lock = Any()

    @Volatile
    private var started = false

    private var latestAccel: FloatArray? = null
    private var latestGyro: FloatArray? = null
    private var latestAccelTimestampNs = 0L
    private var latestGyroTimestampNs = 0L

    fun start() {
        if (started) {
            return
        }
        val accelSensor = accelerometer
            ?: throw IllegalStateException("Accelerometer not available")
        val gyroSensor = gyroscope
            ?: throw IllegalStateException("Gyroscope not available")
        started = true
        sensorManager.registerListener(this, accelSensor, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_GAME)
    }

    fun stop() {
        if (!started) {
            return
        }
        started = false
        sensorManager.unregisterListener(this)
        synchronized(lock) {
            latestAccel = null
            latestGyro = null
            latestAccelTimestampNs = 0L
            latestGyroTimestampNs = 0L
        }
        sampleQueue.clear()
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

                else -> return
            }

            val accel = latestAccel ?: return
            val gyro = latestGyro ?: return
            sampleQueue.offer(
                ImuSample(
                    timestampNs = maxOf(latestAccelTimestampNs, latestGyroTimestampNs),
                    ax = accel[0],
                    ay = accel[1],
                    az = accel[2],
                    gx = gyro[0],
                    gy = gyro[1],
                    gz = gyro[2]
                )
            )
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
