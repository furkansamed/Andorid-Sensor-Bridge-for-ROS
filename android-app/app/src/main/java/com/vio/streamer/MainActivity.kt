package com.vio.streamer

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.widget.Toast
import com.vio.streamer.databinding.ActivityMainBinding
import com.vio.streamer.model.ImuSample
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.regex.Pattern

class MainActivity : Activity(), NetworkSender.Listener {

    private lateinit var binding: ActivityMainBinding
    private val uiHandler = Handler(Looper.getMainLooper())
    private val controlExecutor: ExecutorService = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "stream-control").apply {
            isDaemon = true
        }
    }

    private var networkSender: NetworkSender? = null
    private var cameraStreamer: CameraStreamer? = null
    private var imuCollector: ImuCollector? = null
    private var streamState = StreamState.IDLE
    private var statusMessage = "Idle"

    private var lastStatusUpdateMs = 0L
    private var lastVideoPackets = 0L
    private var lastImuPackets = 0L
    private var lastStatsSender: NetworkSender? = null

    private val statusUpdater = object : Runnable {
        override fun run() {
            refreshStatusText()
            uiHandler.postDelayed(this, STATUS_UPDATE_INTERVAL_MS)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.startStopButton.setOnClickListener {
            handleTogglePressed()
        }
        binding.previewTexture.addOnLayoutChangeListener { _, _, _, _, _, _, _, _, _ ->
            enforcePreviewAspectRatio()
        }
        binding.previewTexture.post {
            enforcePreviewAspectRatio()
        }

        transitionTo(StreamState.IDLE, "Idle")
        uiHandler.post(statusUpdater)

        if (!hasCameraPermission()) {
            requestCameraPermission()
        }
    }

    override fun onPause() {
        super.onPause()
        if (streamState != StreamState.IDLE && streamState != StreamState.ERROR) {
            stopStreaming()
        }
    }

    override fun onDestroy() {
        uiHandler.removeCallbacks(statusUpdater)
        stopStreaming()
        controlExecutor.shutdown()
        super.onDestroy()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE) {
            if (grantResults.firstOrNull() != PackageManager.PERMISSION_GRANTED) {
                transitionTo(StreamState.ERROR, "Camera permission required")
                showToast("Camera permission is required to stream")
            } else if (streamState == StreamState.ERROR) {
                transitionTo(StreamState.IDLE, "Idle")
            }
        }
    }

    override fun onNetworkError(message: String) {
        uiHandler.post {
            if (streamState != StreamState.STOPPING && streamState != StreamState.IDLE) {
                stopStreaming(message)
            }
        }
    }

    private fun handleTogglePressed() {
        when (streamState) {
            StreamState.IDLE,
            StreamState.ERROR -> startStreaming()

            StreamState.CONNECTING,
            StreamState.STARTING_CAMERA,
            StreamState.STREAMING -> stopStreaming()

            StreamState.STOPPING -> Unit
        }
    }

    private fun startStreaming() {
        if (!hasCameraPermission()) {
            requestCameraPermission()
            return
        }

        val host = binding.ipInput.text.toString().trim()
        if (!IPV4_PATTERN.matcher(host).matches()) {
            showToast("Enter a valid IPv4 address")
            return
        }

        resetStats()
        transitionTo(StreamState.CONNECTING, "Connecting to $host")

        val imuQueue = ConcurrentLinkedQueue<ImuSample>()
        val sender = NetworkSender(host, imuQueue, this)
        val collector = ImuCollector(applicationContext, imuQueue)
        networkSender = sender
        imuCollector = collector
        cameraStreamer = null

        try {
            collector.start()
        } catch (error: Exception) {
            networkSender = null
            imuCollector = null
            resetStats()
            val message = "IMU start failed: ${error.message ?: "unknown error"}"
            transitionTo(StreamState.ERROR, message)
            showToast(message)
            return
        }

        controlExecutor.execute {
            try {
                sender.start()
                if (!collector.awaitWarmup(IMU_WARMUP_TIMEOUT_MS)) {
                    throw IllegalStateException("IMU warmup timed out")
                }
                runOnUiThread {
                    if (networkSender !== sender || streamState != StreamState.CONNECTING) {
                        collector.stop()
                        sender.stop()
                        return@runOnUiThread
                    }
                    transitionTo(StreamState.STARTING_CAMERA, "TCP/UDP connected, IMU synced")
                    startCamera(sender, collector, host)
                }
            } catch (error: Exception) {
                collector.stop()
                sender.stop()
                runOnUiThread {
                    if (networkSender === sender) {
                        networkSender = null
                        imuCollector = null
                    }
                    resetStats()
                    val message = "Connection failed: ${error.message ?: "unknown error"}"
                    transitionTo(StreamState.ERROR, message)
                    showToast(message)
                }
            }
        }
    }

    private fun startCamera(
        sender: NetworkSender,
        collector: ImuCollector,
        host: String
    ) {
        lateinit var streamer: CameraStreamer
        val cameraListener = object : CameraStreamer.Listener {
            override fun onStreamingStarted() {
                runOnUiThread {
                    if (cameraStreamer !== streamer || networkSender !== sender) {
                        return@runOnUiThread
                    }
                    transitionTo(StreamState.STREAMING, "Streaming to $host")
                }
            }

            override fun onStreamingError(message: String) {
                runOnUiThread {
                    if (cameraStreamer === streamer) {
                        stopStreaming(message)
                    }
                }
            }
        }

        streamer = CameraStreamer(
            activity = this,
            textureView = binding.previewTexture,
            networkSender = sender,
            imuCollector = collector,
            listener = cameraListener
        )

        cameraStreamer = streamer
        try {
            streamer.start()
        } catch (error: Exception) {
            cameraStreamer = null
            stopStreaming("Camera start failed: ${error.message ?: "unknown error"}")
        }
    }

    private fun stopStreaming(errorMessage: String? = null) {
        val sender = networkSender
        val streamer = cameraStreamer
        val collector = imuCollector

        if (sender == null && streamer == null && collector == null) {
            resetStats()
            transitionTo(
                if (errorMessage == null) StreamState.IDLE else StreamState.ERROR,
                errorMessage ?: "Idle"
            )
            if (errorMessage != null) {
                showToast(errorMessage)
            }
            return
        }

        networkSender = null
        cameraStreamer = null
        imuCollector = null
        resetStats()
        transitionTo(StreamState.STOPPING, "Stopping")

        controlExecutor.execute {
            collector?.stop()
            streamer?.stop()
            sender?.stop()
            runOnUiThread {
                transitionTo(
                    if (errorMessage == null) StreamState.IDLE else StreamState.ERROR,
                    errorMessage ?: "Idle"
                )
                if (errorMessage != null) {
                    showToast(errorMessage)
                }
            }
        }
    }

    private fun hasCameraPermission(): Boolean {
        return checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestCameraPermission() {
        requestPermissions(arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_REQUEST_CODE)
    }

    private fun enforcePreviewAspectRatio() {
        val width = binding.previewTexture.width
        if (width <= 0) {
            return
        }
        val targetHeight = (width * PREVIEW_ASPECT_HEIGHT) / PREVIEW_ASPECT_WIDTH
        val params = binding.previewTexture.layoutParams
        if (params.height != targetHeight) {
            params.height = targetHeight
            binding.previewTexture.layoutParams = params
        }
    }

    private fun transitionTo(state: StreamState, message: String) {
        streamState = state
        statusMessage = message
        binding.startStopButton.text = if (state == StreamState.IDLE || state == StreamState.ERROR) {
            getString(R.string.start)
        } else {
            getString(R.string.stop)
        }
        binding.startStopButton.isEnabled = state != StreamState.STOPPING
        refreshStatusText()
    }

    private fun refreshStatusText() {
        val sender = networkSender
        val now = SystemClock.elapsedRealtime()
        var videoFps = 0f
        var imuHz = 0f

        if (sender != null) {
            val videoPackets = sender.getVideoPacketsSent()
            val imuPackets = sender.getImuPacketsSent()
            if (sender === lastStatsSender && lastStatusUpdateMs > 0L) {
                val elapsedMs = (now - lastStatusUpdateMs).coerceAtLeast(1L)
                videoFps = (videoPackets - lastVideoPackets) * 1000f / elapsedMs.toFloat()
                imuHz = (imuPackets - lastImuPackets) * 1000f / elapsedMs.toFloat()
            }
            lastStatsSender = sender
            lastVideoPackets = videoPackets
            lastImuPackets = imuPackets
            lastStatusUpdateMs = now
        } else {
            lastStatsSender = null
            lastVideoPackets = 0L
            lastImuPackets = 0L
            lastStatusUpdateMs = now
        }

        binding.statusText.text = getString(
            R.string.status_template,
            statusMessage,
            videoFps,
            imuHz
        )
    }

    private fun resetStats() {
        lastStatsSender = null
        lastVideoPackets = 0L
        lastImuPackets = 0L
        lastStatusUpdateMs = 0L
    }

    private fun showToast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    private enum class StreamState {
        IDLE,
        CONNECTING,
        STARTING_CAMERA,
        STREAMING,
        STOPPING,
        ERROR
    }

    private companion object {
        const val CAMERA_PERMISSION_REQUEST_CODE = 1001
        const val PREVIEW_ASPECT_WIDTH = 4
        const val PREVIEW_ASPECT_HEIGHT = 3
        const val STATUS_UPDATE_INTERVAL_MS = 1_000L
        const val IMU_WARMUP_TIMEOUT_MS = 1_000L
        val IPV4_PATTERN: Pattern = Pattern.compile(
            "^(25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)(\\.(25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)){3}$"
        )
    }
}
