package com.example.pitchapp  // <-- change this if your package name is different

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
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

class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "PitchBLE"
        private const val REQ_CODE_BT = 1001
    }

    private lateinit var txtStatus: TextView
    private lateinit var btnScan: Button
    private lateinit var listDevices: ListView

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var isScanning = false

    private val deviceList = ArrayList<BluetoothDevice>()
    private val deviceNames = ArrayList<String>()
    private lateinit var devicesAdapter: ArrayAdapter<String>

    // Permissions we must request at runtime
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

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        txtStatus = findViewById(R.id.txtStatus)
        btnScan = findViewById(R.id.btnScan)
        listDevices = findViewById(R.id.listDevices)

        devicesAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, deviceNames)
        listDevices.adapter = devicesAdapter

        btnScan.setOnClickListener {
            if (isScanning) {
                stopScan()
            } else {
                startBleFlow()
            }
        }

        // Step 1: ask for runtime permissions
        ensurePermissions()
    }

    // ================= PERMISSIONS =================

    private fun ensurePermissions() {
        val missing = REQUIRED_PERMISSIONS.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missing.isNotEmpty()) {
            txtStatus.text = "Requesting BLE/Location permissions..."
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
                txtStatus.text = "Cannot scan: required permission denied.\n" +
                        "Go to Settings → Apps → Pitch BLE Scanner → Permissions."
                Toast.makeText(
                    this,
                    "Please allow Bluetooth & Location permissions in Settings.",
                    Toast.LENGTH_LONG
                ).show()
            }
        }
    }

    // ================= BLUETOOTH INIT =================

    private fun initBluetooth() {
        val manager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = manager.adapter

        if (bluetoothAdapter == null) {
            txtStatus.text = "No Bluetooth adapter found on this device."
            Toast.makeText(this, "No Bluetooth adapter.", Toast.LENGTH_SHORT).show()
            return
        }

        if (!bluetoothAdapter!!.isEnabled) {
            txtStatus.text = "Bluetooth is OFF – please turn it on in system settings."
            Toast.makeText(this, "Turn Bluetooth ON and reopen the app.", Toast.LENGTH_LONG).show()
            return
        }

        bluetoothLeScanner = bluetoothAdapter!!.bluetoothLeScanner
        txtStatus.text = "Bluetooth ready. Tap 'Scan for BLE Devices'."
    }

    // ================= SCANNING =================

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
            txtStatus.text = "SecurityException: OS blocked scan.\nCheck permissions in Settings."
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

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            super.onScanResult(callbackType, result)
            val device = result.device
            val name = device.name ?: "Unnamed"
            val addr = device.address

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

    override fun onDestroy() {
        super.onDestroy()
        if (isScanning) stopScan()
    }
}