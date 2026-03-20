package com.vio.streamer

import com.vio.streamer.model.ImuSample
import java.io.BufferedOutputStream
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

class NetworkSender(
    private val host: String,
    private val imuQueue: ConcurrentLinkedQueue<ImuSample>,
    private val listener: Listener
) {

    interface Listener {
        fun onNetworkError(message: String)
    }

    private val running = AtomicBoolean(false)
    private val errorNotified = AtomicBoolean(false)
    private val videoPacketsSent = AtomicLong(0L)
    private val imuPacketsSent = AtomicLong(0L)
    private val videoLock = Any()

    @Volatile
    private var tcpSocket: Socket? = null

    @Volatile
    private var videoStream: BufferedOutputStream? = null

    @Volatile
    private var udpSocket: DatagramSocket? = null

    @Volatile
    private var imuThread: Thread? = null

    @Throws(IOException::class)
    @Synchronized
    fun start() {
        if (running.get()) {
            return
        }
        val address = InetAddress.getByName(host)
        try {
            val tcp = Socket()
            tcp.tcpNoDelay = true
            tcp.keepAlive = true
            tcp.connect(InetSocketAddress(address, VIDEO_PORT), CONNECT_TIMEOUT_MS)
            val stream = BufferedOutputStream(tcp.getOutputStream())
            val udp = DatagramSocket()
            udp.connect(address, IMU_PORT)

            tcpSocket = tcp
            videoStream = stream
            udpSocket = udp
            errorNotified.set(false)
            videoPacketsSent.set(0L)
            imuPacketsSent.set(0L)
            running.set(true)
            startImuThread()
        } catch (error: IOException) {
            closeSocketsQuietly()
            throw error
        }
    }

    fun stop() {
        running.set(false)
        imuThread?.interrupt()
        closeSocketsQuietly()
        imuThread?.join(IMU_JOIN_TIMEOUT_MS)
        imuThread = null
        imuQueue.clear()
    }

    fun sendVideoPacket(
        frameTimestampNs: Long,
        payload: ByteArray,
        syncedSample: ImuSample?,
        countInStats: Boolean
    ) {
        if (!running.get() || payload.isEmpty()) {
            return
        }
        val sample = syncedSample ?: EMPTY_SAMPLE
        val packet = ByteBuffer.allocate(VIDEO_PACKET_HEADER_BYTES + payload.size)
            .order(ByteOrder.BIG_ENDIAN)
            .putLong(frameTimestampNs)
            .putLong(sample.timestampNs)
            .putFloat(sample.ax)
            .putFloat(sample.ay)
            .putFloat(sample.az)
            .putFloat(sample.gx)
            .putFloat(sample.gy)
            .putFloat(sample.gz)
            .putFloat(sample.qx)
            .putFloat(sample.qy)
            .putFloat(sample.qz)
            .putFloat(sample.qw)
            .putFloat(sample.pitchRad)
            .putFloat(sample.yawRad)
            .putFloat(sample.rollRad)
            .putInt(payload.size)
            .put(payload)
            .array()

        synchronized(videoLock) {
            val stream = videoStream ?: return
            if (!running.get()) {
                return
            }
            try {
                stream.write(packet)
                stream.flush()
                if (countInStats) {
                    videoPacketsSent.incrementAndGet()
                }
            } catch (error: IOException) {
                notifyNetworkError("Video TCP send failed: ${error.message ?: "I/O error"}")
            }
        }
    }

    fun getVideoPacketsSent(): Long = videoPacketsSent.get()

    fun getImuPacketsSent(): Long = imuPacketsSent.get()

    private fun startImuThread() {
        val thread = Thread({
            while (running.get()) {
                val sample = imuQueue.poll()
                if (sample == null) {
                    try {
                        Thread.sleep(IMU_IDLE_SLEEP_MS)
                    } catch (_: InterruptedException) {
                        break
                    }
                    continue
                }

                val payload = encodeImuSample(sample)
                try {
                    udpSocket?.send(DatagramPacket(payload, payload.size))
                    imuPacketsSent.incrementAndGet()
                } catch (error: IOException) {
                    notifyNetworkError("IMU UDP send failed: ${error.message ?: "I/O error"}")
                    break
                }
            }
        }, "imu-sender")
        thread.isDaemon = true
        imuThread = thread
        thread.start()
    }

    private fun encodeImuSample(sample: ImuSample): ByteArray {
        return ByteBuffer.allocate(IMU_PACKET_BYTES)
            .order(ByteOrder.BIG_ENDIAN)
            .putLong(sample.timestampNs)
            .putFloat(sample.ax)
            .putFloat(sample.ay)
            .putFloat(sample.az)
            .putFloat(sample.gx)
            .putFloat(sample.gy)
            .putFloat(sample.gz)
            .putFloat(sample.qx)
            .putFloat(sample.qy)
            .putFloat(sample.qz)
            .putFloat(sample.qw)
            .putFloat(sample.pitchRad)
            .putFloat(sample.yawRad)
            .putFloat(sample.rollRad)
            .array()
    }

    private fun notifyNetworkError(message: String) {
        if (!errorNotified.compareAndSet(false, true)) {
            return
        }
        running.set(false)
        closeSocketsQuietly()
        imuThread?.interrupt()
        listener.onNetworkError(message)
    }

    private fun closeSocketsQuietly() {
        try {
            videoStream?.flush()
        } catch (_: IOException) {
        }
        try {
            videoStream?.close()
        } catch (_: IOException) {
        }
        try {
            tcpSocket?.close()
        } catch (_: IOException) {
        }
        udpSocket?.close()
        videoStream = null
        tcpSocket = null
        udpSocket = null
    }

    private companion object {
        const val VIDEO_PORT = 5000
        const val IMU_PORT = 5001
        const val CONNECT_TIMEOUT_MS = 5_000
        const val IMU_JOIN_TIMEOUT_MS = 500L
        const val IMU_IDLE_SLEEP_MS = 2L
        const val VIDEO_PACKET_HEADER_BYTES = 72
        const val IMU_PACKET_BYTES = 60
        val EMPTY_SAMPLE = ImuSample(
            timestampNs = 0L,
            ax = 0f,
            ay = 0f,
            az = 0f,
            gx = 0f,
            gy = 0f,
            gz = 0f,
            qx = 0f,
            qy = 0f,
            qz = 0f,
            qw = 1f,
            pitchRad = 0f,
            yawRad = 0f,
            rollRad = 0f
        )
    }
}
