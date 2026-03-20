package com.vio.streamer

import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Range
import android.util.Size
import android.view.Surface
import android.view.TextureView
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean

class CameraStreamer(
    private val activity: Activity,
    private val textureView: TextureView,
    private val networkSender: NetworkSender,
    private val imuCollector: ImuCollector,
    private val listener: Listener
) {

    interface Listener {
        fun onStreamingStarted()
        fun onStreamingError(message: String)
    }

    private val cameraManager =
        activity.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private val stopping = AtomicBoolean(false)
    private val errorDelivered = AtomicBoolean(false)
    private val startedDelivered = AtomicBoolean(false)

    @Volatile
    private var cameraThread: HandlerThread? = null

    @Volatile
    private var cameraHandler: Handler? = null

    @Volatile
    private var cameraDevice: CameraDevice? = null

    @Volatile
    private var captureSession: CameraCaptureSession? = null

    @Volatile
    private var previewSurface: Surface? = null

    @Volatile
    private var encoderSurface: Surface? = null

    @Volatile
    private var mediaCodec: MediaCodec? = null

    @Volatile
    private var drainThread: Thread? = null

    @Volatile
    private var drainRunning = false

    @Volatile
    private var encoderToElapsedRealtimeOffsetNs = 0L

    private var textureListener: TextureView.SurfaceTextureListener? = null

    @Throws(Exception::class)
    fun start() {
        stopping.set(false)
        errorDelivered.set(false)
        startedDelivered.set(false)
        encoderToElapsedRealtimeOffsetNs = SystemClock.elapsedRealtimeNanos() - System.nanoTime()
        startCameraThread()
        prepareEncoder()
        if (textureView.isAvailable) {
            openCamera(textureView.surfaceTexture)
        } else {
            waitForTexture()
        }
    }

    fun stop() {
        stopping.set(true)
        textureListener?.let { listener ->
            if (textureView.surfaceTextureListener === listener) {
                textureView.surfaceTextureListener = null
            }
        }
        textureListener = null

        captureSession?.let { session ->
            try {
                session.stopRepeating()
            } catch (_: Exception) {
            }
            try {
                session.abortCaptures()
            } catch (_: Exception) {
            }
            session.close()
        }
        captureSession = null

        cameraDevice?.close()
        cameraDevice = null

        previewSurface?.release()
        previewSurface = null

        drainRunning = false
        drainThread?.interrupt()
        drainThread?.join(DRAIN_JOIN_TIMEOUT_MS)
        drainThread = null

        mediaCodec?.let { codec ->
            try {
                codec.stop()
            } catch (_: Exception) {
            }
            codec.release()
        }
        mediaCodec = null

        encoderSurface?.release()
        encoderSurface = null

        cameraThread?.quitSafely()
        cameraThread?.join(CAMERA_THREAD_JOIN_TIMEOUT_MS)
        cameraThread = null
        cameraHandler = null
    }

    private fun waitForTexture() {
        val listener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
                textureView.surfaceTextureListener = null
                textureListener = null
                openCamera(surface)
            }

            override fun onSurfaceTextureSizeChanged(
                surface: SurfaceTexture,
                width: Int,
                height: Int
            ) = Unit

            override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean = true

            override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit
        }
        textureListener = listener
        textureView.surfaceTextureListener = listener
    }

    private fun startCameraThread() {
        val thread = HandlerThread("camera")
        thread.start()
        cameraThread = thread
        cameraHandler = Handler(thread.looper)
    }

    private fun prepareEncoder() {
        val codec = MediaCodec.createEncoderByType(MIME_TYPE)
        val format = MediaFormat.createVideoFormat(MIME_TYPE, VIDEO_SIZE.width, VIDEO_SIZE.height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, BITRATE_BPS)
            setInteger(MediaFormat.KEY_FRAME_RATE, FRAME_RATE)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, I_FRAME_INTERVAL_SECONDS)
        }
        codec.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        encoderSurface = codec.createInputSurface()
        codec.start()
        mediaCodec = codec
        startDrainThread(codec)
    }

    @SuppressLint("MissingPermission")
    private fun openCamera(surfaceTexture: SurfaceTexture?) {
        if (stopping.get()) {
            return
        }
        val targetTexture = surfaceTexture ?: run {
            dispatchError("TextureView surface is unavailable")
            return
        }
        val cameraId = findBackCameraId() ?: run {
            dispatchError("No usable back camera found")
            return
        }
        targetTexture.setDefaultBufferSize(VIDEO_SIZE.width, VIDEO_SIZE.height)
        try {
            cameraManager.openCamera(cameraId, cameraStateCallback(targetTexture), cameraHandler)
        } catch (error: Exception) {
            dispatchError("Failed to open camera: ${error.message ?: "unknown error"}")
        }
    }

    private fun cameraStateCallback(surfaceTexture: SurfaceTexture): CameraDevice.StateCallback {
        return object : CameraDevice.StateCallback() {
            override fun onOpened(device: CameraDevice) {
                if (stopping.get()) {
                    device.close()
                    return
                }
                cameraDevice = device
                createCaptureSession(device, surfaceTexture)
            }

            override fun onDisconnected(device: CameraDevice) {
                device.close()
                cameraDevice = null
                dispatchError("Camera disconnected")
            }

            override fun onError(device: CameraDevice, error: Int) {
                device.close()
                cameraDevice = null
                dispatchError("Camera error code $error")
            }
        }
    }

    private fun createCaptureSession(device: CameraDevice, surfaceTexture: SurfaceTexture) {
        val encoderInputSurface = encoderSurface ?: run {
            dispatchError("Encoder surface is unavailable")
            return
        }
        val previewOutput = Surface(surfaceTexture)
        previewSurface = previewOutput
        try {
            device.createCaptureSession(
                listOf(previewOutput, encoderInputSurface),
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        if (stopping.get()) {
                            session.close()
                            return
                        }
                        captureSession = session
                        try {
                            val request = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
                                addTarget(previewOutput)
                                addTarget(encoderInputSurface)
                                set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                                set(
                                    CaptureRequest.CONTROL_AF_MODE,
                                    CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
                                )
                                chooseFpsRange(cameraManager.getCameraCharacteristics(device.id))?.let { fpsRange ->
                                    set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
                                }
                            }
                            session.setRepeatingRequest(request.build(), null, cameraHandler)
                            if (startedDelivered.compareAndSet(false, true) && !stopping.get()) {
                                listener.onStreamingStarted()
                            }
                        } catch (error: Exception) {
                            dispatchError("Failed to start camera capture: ${error.message ?: "unknown error"}")
                        }
                    }

                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        session.close()
                        dispatchError("Camera session configuration failed")
                    }
                },
                cameraHandler
            )
        } catch (error: Exception) {
            dispatchError("Failed to create camera session: ${error.message ?: "unknown error"}")
        }
    }

    private fun chooseFpsRange(characteristics: CameraCharacteristics): Range<Int>? {
        val ranges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?: return null
        return ranges
            .filter { it.upper >= FRAME_RATE }
            .sortedWith(compareBy<Range<Int>> { if (it.upper == FRAME_RATE) 0 else 1 }.thenBy { it.lower })
            .firstOrNull()
    }

    private fun findBackCameraId(): String? {
        var fallbackId: String? = null
        for (cameraId in cameraManager.cameraIdList) {
            val characteristics = cameraManager.getCameraCharacteristics(cameraId)
            if (characteristics.get(CameraCharacteristics.LENS_FACING) != CameraCharacteristics.LENS_FACING_BACK) {
                continue
            }
            fallbackId = fallbackId ?: cameraId
            val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: continue
            val supportedPreview = map.getOutputSizes(SurfaceTexture::class.java)
                ?.any { it.width == VIDEO_SIZE.width && it.height == VIDEO_SIZE.height }
                ?: false
            if (supportedPreview) {
                return cameraId
            }
        }
        return fallbackId
    }

    private fun startDrainThread(codec: MediaCodec) {
        drainRunning = true
        val thread = Thread({
            val bufferInfo = MediaCodec.BufferInfo()
            var codecConfigSent = false

            while (drainRunning && !stopping.get()) {
                val outputIndex = try {
                    codec.dequeueOutputBuffer(bufferInfo, DEQUEUE_TIMEOUT_US)
                } catch (_: IllegalStateException) {
                    break
                }

                when {
                    outputIndex == MediaCodec.INFO_TRY_AGAIN_LATER -> continue

                    outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                        if (!codecConfigSent) {
                            val configBytes = extractCodecConfig(codec.outputFormat)
                            if (configBytes.isNotEmpty()) {
                                networkSender.sendVideoPacket(
                                    frameTimestampNs = 0L,
                                    payload = configBytes,
                                    syncedSample = null,
                                    countInStats = false
                                )
                                codecConfigSent = true
                            }
                        }
                    }

                    outputIndex >= 0 -> {
                        val outputBuffer = codec.getOutputBuffer(outputIndex)
                        if (outputBuffer != null && bufferInfo.size > 0) {
                            val packet = outputBuffer.toByteArray(bufferInfo.offset, bufferInfo.size)
                            val isCodecConfig =
                                bufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0
                            val frameTimestampNs =
                                bufferInfo.presentationTimeUs * 1_000L + encoderToElapsedRealtimeOffsetNs
                            val syncedSample = if (isCodecConfig) {
                                null
                            } else {
                                imuCollector.getFrameSyncedSample(frameTimestampNs)
                            }
                            if (isCodecConfig) {
                                codecConfigSent = true
                            }
                            networkSender.sendVideoPacket(
                                frameTimestampNs = frameTimestampNs,
                                payload = packet,
                                syncedSample = syncedSample,
                                countInStats = !isCodecConfig
                            )
                        }
                        codec.releaseOutputBuffer(outputIndex, false)
                    }
                }
            }
        }, "codec-drain")
        thread.isDaemon = true
        drainThread = thread
        thread.start()
    }

    private fun extractCodecConfig(format: MediaFormat): ByteArray {
        val sps = format.getByteBuffer("csd-0")?.toArray()
        val pps = format.getByteBuffer("csd-1")?.toArray()
        if (sps == null && pps == null) {
            return ByteArray(0)
        }
        return when {
            sps == null -> pps!!
            pps == null -> sps
            else -> ByteArray(sps.size + pps.size).apply {
                System.arraycopy(sps, 0, this, 0, sps.size)
                System.arraycopy(pps, 0, this, sps.size, pps.size)
            }
        }
    }

    private fun ByteBuffer.toArray(): ByteArray {
        val duplicate = duplicate()
        val bytes = ByteArray(duplicate.remaining())
        duplicate.get(bytes)
        return bytes
    }

    private fun ByteBuffer.toByteArray(offset: Int, size: Int): ByteArray {
        val duplicate = duplicate()
        duplicate.position(offset)
        duplicate.limit(offset + size)
        val bytes = ByteArray(size)
        duplicate.get(bytes)
        return bytes
    }

    private fun dispatchError(message: String) {
        if (stopping.get()) {
            return
        }
        if (errorDelivered.compareAndSet(false, true)) {
            listener.onStreamingError(message)
        }
    }

    private companion object {
        const val MIME_TYPE = "video/avc"
        val VIDEO_SIZE = Size(640, 480)
        const val FRAME_RATE = 30
        const val BITRATE_BPS = 2_000_000
        const val I_FRAME_INTERVAL_SECONDS = 2
        const val DEQUEUE_TIMEOUT_US = 10_000L
        const val DRAIN_JOIN_TIMEOUT_MS = 500L
        const val CAMERA_THREAD_JOIN_TIMEOUT_MS = 500L
    }
}
