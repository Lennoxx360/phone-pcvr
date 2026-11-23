/* (C)2026 */
package io.github.lennoxx360.phonepcvr

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.widget.LinearLayout
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.TextView
import android.widget.Toast
import androidx.annotation.ColorRes
import androidx.annotation.StringRes
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.switchmaterial.SwitchMaterial
import com.google.android.material.textfield.TextInputEditText
import com.google.android.material.textfield.TextInputLayout
import com.google.ar.core.ArCoreApk
import java.net.Inet4Address

/** Launcher screen: readiness checks, settings and the Start button. */
class HomeActivity : AppCompatActivity() {

    private lateinit var checks: LinearLayout
    private val handler = Handler(Looper.getMainLooper())
    private val prefs by lazy { getSharedPreferences(Settings.PREFS, MODE_PRIVATE) }
    private val switches = mutableMapOf<Settings.Toggle, SwitchMaterial>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_home)
        checks = findViewById(R.id.checks)

        findViewById<TextView>(R.id.version).text =
            getString(R.string.version_fmt, BuildConfig.VERSION_NAME)
        findViewById<View>(R.id.start_vr).setOnClickListener { startVr(scanViewer = false) }
        // Testing hook (adb): am start -n <pkg>/.HomeActivity --ez start_vr true
        //   [--es stream_profile <id>] [--ez <toggle key> true|false] [--ef <tune key> <value>]
        handleTestIntent(intent)
        findViewById<View>(R.id.scan_viewer).setOnClickListener { startVr(scanViewer = true) }
        findViewById<View>(R.id.about).setOnClickListener {
            startActivity(Intent(this, AboutActivity::class.java))
        }

        addToggles(findViewById(R.id.settings_tracking), Settings.TRACKING)
        addToggles(findViewById(R.id.settings_display), Settings.DISPLAY)
        addStreamingSettings(findViewById(R.id.settings_streaming))
        addFitSteppers(findViewById(R.id.settings_fit))
        addToggles(
            findViewById(R.id.settings_fit_keys),
            listOf(
                Settings.Toggle(
                    Settings.FIT_VOLUME_KEYS,
                    true,
                    R.string.fit_volume_keys,
                    R.string.fit_volume_keys_desc)))
        addToggles(
            findViewById(R.id.settings_play_space),
            listOf(
                Settings.Toggle(
                    Settings.RECENTER_ON_START,
                    false,
                    R.string.recenter_on_start,
                    R.string.recenter_on_start_desc)))
        // The phone isn't on your head here, so queue it for the next stream.
        findViewById<View>(R.id.reset_play_space).setOnClickListener {
            prefs.edit().putBoolean(Settings.RECENTER_PENDING, true).apply()
            Toast.makeText(this, R.string.reset_play_space_queued, Toast.LENGTH_LONG).show()
        }
        findViewById<View>(R.id.fit_reset).setOnClickListener {
            Settings.resetFit(prefs)
            syncFit()
        }
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        handleTestIntent(intent)
    }

    private fun handleTestIntent(intent: Intent?) {
        if (intent?.getBooleanExtra("start_vr", false) != true) return
        val vr = Intent(this, ALVRActivity::class.java)
        intent.extras?.let { vr.putExtras(it) }
        startActivity(vr)
    }

    // ---- Headset fit ----

    private val fitValues = mutableMapOf<Settings.Fit, TextView>()

    private fun addFitSteppers(container: LinearLayout) {
        val inflater = LayoutInflater.from(this)
        for (f in Settings.FIT) {
            val row = inflater.inflate(R.layout.item_stepper, container, false)
            row.findViewById<TextView>(R.id.title).setText(f.title)
            row.findViewById<TextView>(R.id.desc).setText(f.desc)
            fitValues[f] = row.findViewById(R.id.value)
            row.findViewById<View>(R.id.minus).setOnClickListener {
                f.step(prefs, -1)
                syncFit()
            }
            row.findViewById<View>(R.id.plus).setOnClickListener {
                f.step(prefs, +1)
                syncFit()
            }
            container.addView(row)
        }
        syncFit()
    }

    private fun syncFit() {
        for ((f, tv) in fitValues) tv.text = f.format(this, prefs)
        for ((t, tv) in tuneValues) tv.text = t.format(this, prefs)
        profileValue?.let {
            val id = Settings.currentProfile(prefs)
            it.text =
                getString(R.string.stream_profile) + ": " + getString(Settings.profileTitle(id))
            profileDesc?.setText(Settings.profileDesc(id))
        }
    }

    // ---- Streaming: latency & picture ----

    private val tuneValues = mutableMapOf<Settings.Tune, TextView>()
    private var profileValue: TextView? = null
    private var profileDesc: TextView? = null

    private fun addStreamingSettings(container: LinearLayout) {
        val inflater = LayoutInflater.from(this)
        val note = TextView(this, null, 0, R.style.Pcvr_Caption)
        note.setText(R.string.streaming_note)
        container.addView(note)

        // Profile: a tappable row with the current choice as its "value".
        val row = inflater.inflate(R.layout.item_setting, container, false)
        row.findViewById<View>(R.id.toggle).visibility = View.GONE
        profileValue = row.findViewById(R.id.title)
        profileDesc = row.findViewById(R.id.desc)
        row.setOnClickListener { showProfileDialog() }
        container.addView(row)

        addToggles(container, Settings.STREAM + Settings.EXPERIMENTAL)
        for (t in Settings.TUNE + Settings.EXPERIMENTAL_TUNE) {
            val r = inflater.inflate(R.layout.item_stepper, container, false)
            r.findViewById<TextView>(R.id.title).setText(t.title)
            r.findViewById<TextView>(R.id.desc).setText(t.desc)
            tuneValues[t] = r.findViewById(R.id.value)
            r.findViewById<View>(R.id.minus).setOnClickListener {
                t.step(prefs, -1)
                syncFit()
            }
            r.findViewById<View>(R.id.plus).setOnClickListener {
                t.step(prefs, +1)
                syncFit()
            }
            container.addView(r)
        }
        syncFit()
    }

    private fun showProfileDialog() {
        val ids = Settings.PROFILE_IDS
        val labels =
            ids.map {
                    getString(Settings.profileTitle(it)) +
                        "\n" +
                        getString(Settings.profileDesc(it))
                }
                .toTypedArray()
        val current = ids.indexOf(Settings.currentProfile(prefs))
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.stream_profile)
            .setSingleChoiceItems(labels, current) { d, which ->
                Settings.applyProfile(prefs, ids[which])
                syncToggles()
                syncFit()
                Toast.makeText(this, R.string.stream_options_restart, Toast.LENGTH_SHORT).show()
                d.dismiss()
            }
            .setNegativeButton(android.R.string.cancel, null)
            .show()
    }

    override fun onResume() {
        super.onResume()
        // The in-VR ⚙ menu writes the same prefs.
        syncToggles()
        syncFit()
        refreshChecks()
    }

    override fun onPause() {
        super.onPause()
        handler.removeCallbacksAndMessages(null)
    }

    private fun startVr(scanViewer: Boolean) {
        val intent = Intent(this, ALVRActivity::class.java)
        if (scanViewer) intent.putExtra(ALVRActivity.EXTRA_SCAN_VIEWER, true)
        startActivity(intent)
    }

    // ---- Settings ----

    private fun addToggles(container: LinearLayout, toggles: List<Settings.Toggle>) {
        val inflater = LayoutInflater.from(this)
        for (t in toggles) {
            val row = inflater.inflate(R.layout.item_setting, container, false)
            row.findViewById<TextView>(R.id.title).setText(t.title)
            row.findViewById<TextView>(R.id.desc).setText(t.desc)
            val sw = row.findViewById<SwitchMaterial>(R.id.toggle)
            switches[t] = sw
            row.setOnClickListener {
                sw.isChecked = !sw.isChecked
                val e = prefs.edit().putBoolean(t.key, sw.isChecked)
                if (Settings.isStreamToggle(t.key))
                    e.putString(Settings.STREAM_PROFILE, Settings.PROFILE_CUSTOM)
                e.apply()
                if (Settings.isStreamToggle(t.key)) syncFit()
                if (t.key == Settings.ARCORE_6DOF) refreshChecks()
            }
            container.addView(row)
        }
        syncToggles()
    }

    private fun syncToggles() {
        for ((t, sw) in switches) sw.isChecked = prefs.getBoolean(t.key, t.default)
    }

    // ---- Readiness checks ----
    // Every row opens its own settings, whatever its status.

    private enum class State(@ColorRes val color: Int) {
        OK(R.color.pcvr_ok),
        WARN(R.color.pcvr_warn),
        BAD(R.color.pcvr_bad),
        INFO(R.color.pcvr_cyan)
    }

    private fun refreshChecks() {
        handler.removeCallbacksAndMessages(null)
        checks.removeAllViews()
        CrashLog.lastCrash(this)?.let { file ->
            addCheck(getString(R.string.app_crash_toast), file.name, State.BAD) {
                showCrashDialog()
            }
        }
        checkNetwork()
        checkCamera()
        checkArCore()
        addCheck(getString(R.string.check_pc), getString(R.string.check_pc_desc), State.INFO) {
            showPcDialog()
        }
    }

    private fun addCheck(title: String, desc: String, state: State, onClick: () -> Unit) {
        val row = LayoutInflater.from(this).inflate(R.layout.item_check, checks, false)
        row.findViewById<TextView>(R.id.title).text = title
        row.findViewById<TextView>(R.id.desc).text = desc
        row.findViewById<View>(R.id.dot).backgroundTintList =
            ColorStateList.valueOf(ContextCompat.getColor(this, state.color))
        row.setOnClickListener { onClick() }
        checks.addView(row)
    }

    private fun addCheck(
        @StringRes title: Int,
        @StringRes desc: Int,
        state: State,
        onClick: () -> Unit
    ) = addCheck(getString(title), getString(desc), state, onClick)

    // ---- Network ----

    /** Wi-Fi may not be the default network (a hotspot has no internet), so look at them all. */
    private fun wifiIp(): String? {
        val cm = getSystemService(ConnectivityManager::class.java)
        @Suppress("DEPRECATION")
        for (net in cm.allNetworks) {
            val caps = cm.getNetworkCapabilities(net) ?: continue
            if (!caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) continue
            val ip =
                cm.getLinkProperties(net)
                    ?.linkAddresses
                    ?.map { it.address }
                    ?.firstOrNull { it is Inet4Address }
                    ?.hostAddress
            if (ip != null) return ip
        }
        return null
    }

    private fun checkNetwork() {
        val ip = wifiIp()
        val manual = prefs.getBoolean(Settings.NETWORK_MANUAL, false)
        val pcIp = prefs.getString(Settings.PC_IP, "") ?: ""
        val title = getString(R.string.check_network)
        val onClick = { showNetworkDialog() }
        when {
            manual && ip == null ->
                addCheck(
                    title, getString(R.string.check_network_manual_noip, pcIp), State.BAD, onClick)
            manual ->
                addCheck(
                    title, getString(R.string.check_network_manual, pcIp, ip), State.OK, onClick)
            ip == null ->
                addCheck(title, getString(R.string.check_network_none), State.BAD, onClick)
            // Windows Mobile hotspot hands out 192.168.137.x.
            ip.startsWith("192.168.137.") ->
                addCheck(title, getString(R.string.check_network_hotspot, ip), State.OK, onClick)
            else -> addCheck(title, getString(R.string.check_network_wifi, ip), State.OK, onClick)
        }
    }

    private fun showNetworkDialog() {
        val view = LayoutInflater.from(this).inflate(R.layout.dialog_network, null)
        val auto = view.findViewById<RadioButton>(R.id.mode_auto)
        val manual = view.findViewById<RadioButton>(R.id.mode_manual)
        val ipLayout = view.findViewById<TextInputLayout>(R.id.pc_ip_layout)
        val ipEdit = view.findViewById<TextInputEditText>(R.id.pc_ip)
        val hotspot = view.findViewById<View>(R.id.use_hotspot_ip)

        val isManual = prefs.getBoolean(Settings.NETWORK_MANUAL, false)
        auto.isChecked = !isManual
        manual.isChecked = isManual
        ipEdit.setText(prefs.getString(Settings.PC_IP, ""))
        fun syncEnabled() {
            ipLayout.isEnabled = manual.isChecked
            hotspot.isEnabled = manual.isChecked
            if (!manual.isChecked) ipLayout.error = null
        }
        syncEnabled()
        view.findViewById<RadioGroup>(R.id.mode).setOnCheckedChangeListener { _, _ ->
            syncEnabled()
        }
        hotspot.setOnClickListener { ipEdit.setText(HOTSPOT_PC_IP) }

        view.findViewById<TextView>(R.id.this_phone).text =
            getString(
                R.string.network_this_phone,
                wifiIp() ?: getString(R.string.unknown),
                prefs.getString(Settings.ALVR_HOSTNAME, null)
                    ?: getString(R.string.hostname_unknown))
        view.findViewById<View>(R.id.wifi_settings).setOnClickListener { openWifiSettings() }

        val dialog =
            MaterialAlertDialogBuilder(this)
                .setTitle(R.string.check_network)
                .setView(view)
                .setPositiveButton(R.string.save, null)
                .setNegativeButton(android.R.string.cancel, null)
                .show()
        // Validate before closing.
        dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener {
            val ip = ipEdit.text?.toString()?.trim() ?: ""
            if (manual.isChecked && !ManualAnnouncer.isValidIp(ip)) {
                ipLayout.error = getString(R.string.pc_ip_invalid)
                return@setOnClickListener
            }
            prefs
                .edit()
                .putBoolean(Settings.NETWORK_MANUAL, manual.isChecked)
                .putString(Settings.PC_IP, ip)
                .apply()
            dialog.dismiss()
            refreshChecks()
        }
    }

    private fun openWifiSettings() {
        val intent =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q)
                Intent(android.provider.Settings.Panel.ACTION_WIFI)
            else Intent(android.provider.Settings.ACTION_WIFI_SETTINGS)
        tryStart(intent)
    }

    // ---- Camera ----

    private fun hasCamera() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    private fun checkCamera() {
        if (hasCamera()) {
            addCheck(R.string.check_camera, R.string.check_camera_ok, State.OK) {
                showCameraDialog()
            }
        } else {
            addCheck(R.string.check_camera, R.string.check_camera_missing, State.WARN) {
                showCameraDialog()
            }
        }
    }

    private fun showCameraDialog() {
        val granted = hasCamera()
        val b =
            MaterialAlertDialogBuilder(this)
                .setTitle(R.string.check_camera)
                .setMessage(
                    if (granted) R.string.camera_dialog_granted else R.string.camera_dialog_missing)
                .setPositiveButton(R.string.open_app_permissions) { _, _ -> openAppSettings() }
                .setNegativeButton(R.string.close, null)
        if (!granted) {
            b.setNeutralButton(R.string.grant) { _, _ ->
                ActivityCompat.requestPermissions(
                    this, arrayOf(Manifest.permission.CAMERA), CAMERA_REQUEST)
            }
        }
        b.show()
    }

    private fun openAppSettings() {
        tryStart(
            Intent(android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS)
                .setData(Uri.fromParts("package", packageName, null)))
    }

    // ---- ARCore ----

    private fun checkArCore() {
        val onClick = { showArCoreDialog() }
        if (!prefs.getBoolean(Settings.ARCORE_6DOF, true)) {
            addCheck(R.string.check_arcore, R.string.check_arcore_off, State.INFO, onClick)
            return
        }
        val availability = ArCoreApk.getInstance().checkAvailability(this)
        when {
            availability.isTransient -> {
                addCheck(R.string.check_arcore, R.string.check_arcore_checking, State.INFO, onClick)
                handler.postDelayed({ refreshChecks() }, 400)
            }
            availability == ArCoreApk.Availability.SUPPORTED_INSTALLED ->
                addCheck(R.string.check_arcore, R.string.check_arcore_ok, State.OK, onClick)
            availability.isSupported ->
                addCheck(R.string.check_arcore, R.string.check_arcore_install, State.WARN, onClick)
            else ->
                addCheck(
                    R.string.check_arcore, R.string.check_arcore_unsupported, State.WARN, onClick)
        }
    }

    private fun showArCoreDialog() {
        val on = prefs.getBoolean(Settings.ARCORE_6DOF, true)
        val availability = ArCoreApk.getInstance().checkAvailability(this)
        val status =
            when {
                availability == ArCoreApk.Availability.SUPPORTED_INSTALLED ->
                    getString(R.string.check_arcore_ok)
                availability.isTransient -> getString(R.string.check_arcore_checking)
                availability.isSupported -> getString(R.string.check_arcore_install)
                else -> getString(R.string.check_arcore_unsupported)
            }
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.check_arcore)
            .setMessage(getString(R.string.arcore_dialog, status))
            .setPositiveButton(if (on) R.string.turn_6dof_off else R.string.turn_6dof_on) { _, _ ->
                prefs.edit().putBoolean(Settings.ARCORE_6DOF, !on).apply()
                syncToggles()
                refreshChecks()
            }
            .setNeutralButton(R.string.arcore_install_update) { _, _ ->
                try {
                    ArCoreApk.getInstance().requestInstall(this, true)
                } catch (e: Exception) {
                    Log.w(TAG, "ARCore install request failed, opening the store page", e)
                    openArCoreStorePage()
                }
            }
            .setNegativeButton(R.string.close, null)
            .show()
    }

    private fun openArCoreStorePage() {
        if (!tryStart(Intent(Intent.ACTION_VIEW, Uri.parse("market://details?id=$ARCORE_PKG")))) {
            tryStart(
                Intent(
                    Intent.ACTION_VIEW,
                    Uri.parse("https://play.google.com/store/apps/details?id=$ARCORE_PKG")))
        }
    }

    // ---- PC streamer ----

    private fun showPcDialog() {
        val hostname = prefs.getString(Settings.ALVR_HOSTNAME, null)
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.check_pc)
            .setMessage(
                getString(
                    R.string.pc_dialog,
                    hostname ?: getString(R.string.hostname_unknown),
                    wifiIp() ?: getString(R.string.unknown)))
            .setPositiveButton(R.string.network_settings) { _, _ -> showNetworkDialog() }
            .setNeutralButton(R.string.alvr_download) { _, _ ->
                tryStart(Intent(Intent.ACTION_VIEW, Uri.parse(ALVR_RELEASE_URL)))
            }
            .setNegativeButton(R.string.close, null)
            .show()
    }

    // ---- Shared ----

    private fun tryStart(intent: Intent): Boolean =
        try {
            startActivity(intent)
            true
        } catch (e: Exception) {
            Log.w(TAG, "No activity for $intent", e)
            false
        }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        refreshChecks()
    }

    private fun showCrashDialog() {
        val file = CrashLog.lastCrash(this) ?: return
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.app_crash_toast)
            .setMessage(file.readText().take(4000))
            .setPositiveButton(R.string.crash_share) { _, _ -> CrashLog.share(this, file) }
            .setNegativeButton(R.string.crash_delete) { _, _ ->
                file.delete()
                refreshChecks()
            }
            .setNeutralButton(android.R.string.cancel, null)
            .show()
    }

    companion object {
        private const val TAG = "PVR-Home"
        private const val CAMERA_REQUEST = 1
        /** Windows Mobile hotspot's own address. */
        private const val HOTSPOT_PC_IP = "192.168.137.1"
        private const val ARCORE_PKG = "com.google.ar.core"
        private const val ALVR_RELEASE_URL = "https://github.com/alvr-org/ALVR/releases/tag/v20.8.0"
    }
}
