package com.example.pitchapp   // make sure this matches your package name

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import org.json.JSONObject
import java.nio.charset.Charset
import java.util.UUID
import kotlin.math.abs

class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "PitchBLE"
        private const val REQ_CODE_BT = 1001

        // Nordic UART-style service / chars. MUST match ESP32 firmware.
        val UART_SERVICE_UUID: UUID =
            UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val UART_RX_CHAR_UUID: UUID =
            UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E") // notify (ESP32 -> app)
        val UART_TX_CHAR_UUID: UUID =
            UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // write (app -> ESP32)

        val CCCD_UUID: UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        // For filtering scan results to ESP tuner
        private const val ESP_DEVICE_NAME = "ESP32-YinPitch-44k"
        private const val ESP_NAME_PREFIX = "ESP32"
        private const val ESP_NAME_TOKEN = "YinPitch"
    }

    // ----- UI -----
    private lateinit var txtStatus: TextView
    private lateinit var btnScan: Button
    private lateinit var listDevices: ListView

    private lateinit var txtCurrentNote: TextView
    private lateinit var txtCurrentFreq: TextView
    private lateinit var txtCurrentCents: TextView
    private lateinit var txtCurrentProb: TextView

    private lateinit var edtNote: EditText
    private lateinit var btnPlayNote: Button
    private lateinit var btnMute: Button
    private lateinit var txtPlayStatus: TextView

    private lateinit var btnToggleLog: Button
    private lateinit var txtLogResult: TextView

    // ----- BLE base -----
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var isScanning = false

    private val deviceList = ArrayList<BluetoothDevice>()
    private val deviceNames = ArrayList<String>()
    private lateinit var devicesAdapter: ArrayAdapter<String>

    // ----- Connection / GATT -----
    private var connectedDevice: BluetoothDevice? = null
    private var bluetoothGatt: BluetoothGatt? = null
    private var uartRxChar: BluetoothGattCharacteristic? = null  // notify
    private var uartTxChar: BluetoothGattCharacteristic? = null  // write

    // ----- Permissions -----
    private val REQUIRED_PERMISSIONS: Array<String> by lazy {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
    }

    // ----- Accuracy logging -----
    private var logging = false
    private var logStartTime: Long = 0L
    private val logCents = ArrayList<Float>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Bind UI
        txtStatus = findViewById(R.id.txtStatus)
        btnScan = findViewById(R.id.btnScan)
        listDevices = findViewById(R.id.listDevices)

        txtCurrentNote = findViewById(R.id.txtCurrentNote)
        txtCurrentFreq = findViewById(R.id.txtCurrentFreq)
        txtCurrentCents = findViewById(R.id.txtCurrentCents)
        txtCurrentProb = findViewById(R.id.txtCurrentProb)

        edtNote = findViewById(R.id.edtNote)
        btnPlayNote = findViewById(R.id.btnPlayNote)
        btnMute = findViewById(R.id.btnMute)
        txtPlayStatus = findViewById(R.id.txtPlayStatus)

        btnToggleLog = findViewById(R.id.btnToggleLog)
        txtLogResult = findViewById(R.id.txtLogResult)

        // Device list adapter
        devicesAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, deviceNames)
        listDevices.adapter = devicesAdapter

        // Tap device → connect
        listDevices.setOnItemClickListener { _, _, position, _ ->
            val dev = deviceList[position]
            stopScan()
            connectToDevice(dev)
        }

        // Scan button
        btnScan.setOnClickListener {
            if (isScanning) {
                stopScan()
            } else {
                startBleFlow()
            }
        }

        // Play note
        btnPlayNote.setOnClickListener {
            val note = edtNote.text.toString().trim()
            if (note.isEmpty()) {
                txtPlayStatus.text = "Enter a note like C4, D4, A4."
                return@setOnClickListener
            }
            sendCommand("PLAY NOTE=$note\n")
        }

        // Mute
        btnMute.setOnClickListener {
            sendCommand("MUTE\n")
        }

        // Accuracy logger
        btnToggleLog.setOnClickListener {
            toggleLogging()
        }

        // Permissions
        ensurePermissions()
    }

    // ============== PERMISSIONS ==============

    private fun ensurePermissions() {
        val missing = REQUIRED_PERMISSIONS.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missing.isNotEmpty()) {
            txtStatus.text = "Requesting BLE / Location permissions..."
            ActivityCompat.requestPermissions(
                this,
                missing.toTypedArray(),
                REQ_CODE_BT
            )
        } else {
            txtStatus.text = "Permissions OK. Initializing Bluetooth..."
            initBluetooth()
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (requestCode == REQ_CODE_BT) {
            var allGranted = true
            for (i in permissions.indices) {
                val granted = grantResults.getOrNull(i) == PackageManager.PERMISSION_GRANTED
                Log.d(TAG, "Permission ${permissions[i]} granted=$granted")
                if (!granted) allGranted = false
            }

            if (allGranted) {
                txtStatus.text = "Permissions granted. Initializing Bluetooth..."
                initBluetooth()
            } else {
                txtStatus.text = "Cannot scan: permission denied.\n" +
                        "Go to Settings → Apps → Pitch BLE Scanner → Permissions."
                Toast.makeText(
                    this,
                    "Please allow Bluetooth & Location permissions in Settings.",
                    Toast.LENGTH_LONG
                ).show()
            }
        }
    }

    // ============== BLUETOOTH INIT ==============

    private fun initBluetooth() {
        val manager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = manager.adapter

        if (bluetoothAdapter == null) {
            txtStatus.text = "No Bluetooth adapter on this device."
            Toast.makeText(this, "No Bluetooth adapter.", Toast.LENGTH_SHORT).show()
            return
        }

        if (!bluetoothAdapter!!.isEnabled) {
            txtStatus.text = "Bluetooth is OFF – turn it on and reopen the app."
            Toast.makeText(this, "Turn Bluetooth ON.", Toast.LENGTH_LONG).show()
            return
        }

        bluetoothLeScanner = bluetoothAdapter!!.bluetoothLeScanner
        txtStatus.text = "Bluetooth ready. Tap 'Scan for BLE Devices'."
    }

    // ============== SCANNING ==============

    private fun startBleFlow() {
        if (bluetoothAdapter == null || bluetoothLeScanner == null) {
            txtStatus.text = "Bluetooth not ready. Re-checking..."
            initBluetooth()
            return
        }
        startScan()
    }

    private fun startScan() {
        if (isScanning) return

        val scanner = bluetoothLeScanner ?: run {
            txtStatus.text = "Scanner is null."
            return
        }

        isScanning = true
        txtStatus.text = "Status: Scanning..."
        btnScan.text = "Stop Scan"

        deviceList.clear()
        deviceNames.clear()
        devicesAdapter.notifyDataSetChanged()

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        try {
            scanner.startScan(null, settings, scanCallback)
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException on startScan: ${e.message}")
            txtStatus.text = "SecurityException: OS blocked scan (check perms)."
            isScanning = false
            btnScan.text = "Scan for BLE Devices"
        }
    }

    private fun stopScan() {
        if (!isScanning) return
        try {
            bluetoothLeScanner?.stopScan(scanCallback)
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException on stopScan: ${e.message}")
        }
        isScanning = false
        txtStatus.text = "Status: Idle"
        btnScan.text = "Scan for BLE Devices"
    }

    // Filter scan results to ESP32 tuner-ish devices
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            super.onScanResult(callbackType, result)
            val device = result.device
            val name = device.name ?: "Unnamed"
            val addr = device.address

            val isEsp =
                name == ESP_DEVICE_NAME ||
                        name.contains(ESP_NAME_TOKEN, ignoreCase = true) ||
                        name.contains(ESP_NAME_PREFIX, ignoreCase = true)

            if (!isEsp) return
            if (deviceList.any { it.address == addr }) return

            Log.d(TAG, "Found device: $name - $addr")
            deviceList.add(device)
            deviceNames.add("$name\n$addr")

            runOnUiThread {
                devicesAdapter.notifyDataSetChanged()
            }
        }

        override fun onScanFailed(errorCode: Int) {
            super.onScanFailed(errorCode)
            Log.e(TAG, "Scan failed: $errorCode")
            isScanning = false
            runOnUiThread {
                txtStatus.text = "Status: Scan failed (code $errorCode)"
                btnScan.text = "Scan for BLE Devices"
            }
        }
    }

    // ============== CONNECT + GATT ==============

    private fun connectToDevice(device: BluetoothDevice) {
        txtStatus.text = "Connecting to ${device.name ?: device.address}..."
        connectedDevice = device

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
            != PackageManager.PERMISSION_GRANTED
        ) {
            txtStatus.text = "No BLUETOOTH_CONNECT permission."
            return
        }

        bluetoothGatt?.close()
        bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        txtPlayStatus.text = ""
    }

    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            super.onConnectionStateChange(gatt, status, newState)

            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "GATT error onConnectionStateChange status=$status")
                runOnUiThread {
                    txtStatus.text = "GATT error: $status"
                }
                gatt.close()
                return
            }

            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.d(TAG, "Connected to GATT, requesting MTU 100...")
                    runOnUiThread {
                        txtStatus.text = "Connected. Requesting MTU..."
                    }
                    // Request bigger MTU so the full JSON fits in one notification.
                    gatt.requestMtu(100)
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.d(TAG, "Disconnected from GATT")
                    runOnUiThread {
                        txtStatus.text = "Disconnected."
                    }
                    uartRxChar = null
                    uartTxChar = null
                    bluetoothGatt?.close()
                    bluetoothGatt = null
                }
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            super.onMtuChanged(gatt, mtu, status)
            Log.d(TAG, "MTU changed: mtu=$mtu status=$status")
            runOnUiThread {
                txtStatus.text = "MTU=$mtu, discovering services..."
            }
            gatt.discoverServices()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            super.onServicesDiscovered(gatt, status)

            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "onServicesDiscovered status=$status")
                runOnUiThread {
                    txtStatus.text = "Service discovery failed: $status"
                }
                return
            }

            val service = gatt.getService(UART_SERVICE_UUID)
            if (service == null) {
                Log.e(TAG, "UART service not found")
                runOnUiThread {
                    txtStatus.text = "UART service not found on device."
                }
                return
            }

            uartRxChar = service.getCharacteristic(UART_RX_CHAR_UUID)
            uartTxChar = service.getCharacteristic(UART_TX_CHAR_UUID)

            if (uartRxChar == null || uartTxChar == null) {
                Log.e(TAG, "UART RX/TX characteristics not found")
                runOnUiThread {
                    txtStatus.text = "UART RX/TX characteristics not found."
                }
                return
            }

            enableNotifications(gatt, uartRxChar!!)
            runOnUiThread {
                txtStatus.text = "Connected & ready. Streaming pitch..."
            }

            // Optional: tell ESP32 to START streaming
            sendCommand("START\n")
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            super.onCharacteristicChanged(gatt, characteristic)
            if (characteristic.uuid == UART_RX_CHAR_UUID) {
                val data = characteristic.value
                val text = data.toString(Charset.forName("UTF-8"))

                // Show raw data on screen for debugging
                runOnUiThread {
                    txtLogResult.text = "Last raw: $text"
                }

                handleJsonString(text)
            }
        }
    }

    private fun enableNotifications(gatt: BluetoothGatt, ch: BluetoothGattCharacteristic) {
        gatt.setCharacteristicNotification(ch, true)
        val desc = ch.getDescriptor(CCCD_UUID)
        if (desc != null) {
            desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(desc)
        }
    }

    // ============== HANDLE JSON FROM ESP32 ==============

    private fun handleJsonString(raw: String) {
        // We expect something like:
        // {"f0":145.53,"note":"D3","cents":-15.4,"rms":0.396,"prob":0.98}
        val trimmed = raw.trim()
        if (!trimmed.startsWith("{") || !trimmed.endsWith("}")) {
            Log.w(TAG, "Not a full JSON object: '$raw'")
            return
        }

        try {
            val obj = JSONObject(trimmed)

            val f0 = obj.optDouble("f0", 0.0)
            val note = obj.optString("note", "--")
            val cents = obj.optDouble("cents", 0.0)
            val rms = obj.optDouble("rms", 0.0)
            val prob = obj.optDouble("prob", 0.0)

            runOnUiThread {
                txtCurrentNote.text = "Note: $note"
                txtCurrentFreq.text = String.format("f0: %.2f Hz", f0)
                txtCurrentCents.text = String.format("Cents: %.1f", cents)
                txtCurrentProb.text = String.format("Confidence: %.2f", prob)
            }

            if (logging && note != "--" && !cents.isNaN()) {
                synchronized(logCents) {
                    logCents.add(abs(cents.toFloat()))
                }
            }

        } catch (e: Exception) {
            Log.e(TAG, "JSON parse error: ${e.message} raw=$raw")
        }
    }

    // ============== SEND COMMANDS (PLAY / MUTE) ==============

    private fun sendCommand(cmd: String) {
        val gatt = bluetoothGatt
        val tx = uartTxChar
        if (gatt == null || tx == null) {
            runOnUiThread {
                txtPlayStatus.text = "Not connected to ESP32."
            }
            return
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
            != PackageManager.PERMISSION_GRANTED
        ) {
            runOnUiThread {
                txtPlayStatus.text = "No BLUETOOTH_CONNECT permission."
            }
            return
        }

        try {
            val bytes = cmd.toByteArray(Charset.forName("UTF-8"))

            val ok: Boolean = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                gatt.writeCharacteristic(
                    tx,
                    bytes,
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                )
                true  // don't rely on return, just assume ok
            } else {
                tx.value = bytes
                tx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                gatt.writeCharacteristic(tx)
            }

            runOnUiThread {
                txtPlayStatus.text = if (ok) "Command sent: $cmd" else "Write failed."
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error writing characteristic: ${e.message}")
            runOnUiThread {
                txtPlayStatus.text = "Error sending command."
            }
        }
    }

    // ============== ACCURACY LOGGING ==============

    private fun toggleLogging() {
        logging = !logging
        if (logging) {
            synchronized(logCents) {
                logCents.clear()
            }
            logStartTime = System.currentTimeMillis()
            btnToggleLog.text = "Stop Accuracy Log"
            txtLogResult.text = "Logging... hold a note on pitch!"
        } else {
            val endTime = System.currentTimeMillis()
            val durationSec = (endTime - logStartTime) / 1000.0

            val samples: List<Float>
            synchronized(logCents) {
                samples = ArrayList(logCents)
            }

            btnToggleLog.text = "Start Accuracy Log"

            if (samples.isEmpty()) {
                txtLogResult.text = "No samples logged."
            } else {
                val avg = samples.sum() / samples.size
                val max = samples.maxOrNull() ?: 0f
                txtLogResult.text = String.format(
                    "Last log:\nDuration: %.1f s\nSamples: %d\nAvg abs error: %.1f cents\nMax abs error: %.1f cents",
                    durationSec, samples.size, avg, max
                )
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopScan()
        bluetoothGatt?.close()
        bluetoothGatt = null
    }
}
