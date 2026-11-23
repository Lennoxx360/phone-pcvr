/* (C)2026 */
package io.github.lennoxx360.phonepcvr

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.util.TypedValue
import android.view.LayoutInflater
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.google.android.material.button.MaterialButton
import com.google.android.material.dialog.MaterialAlertDialogBuilder

/** About, credits for every project Phone PCVR is built on, and the licence texts. */
class AboutActivity : AppCompatActivity() {

    private data class Credit(
        val name: String,
        val by: String,
        val what: String,
        val license: String,
        val url: String
    )

    private val credits =
        listOf(
            Credit(
                "PhoneVR",
                "PhoneVR-Developers and contributors",
                "The Android ALVR client Phone PCVR grew out of: connection, video decoding and " +
                    "Cardboard rendering.",
                "GPL-3.0",
                "https://github.com/PhoneVR-Developers/PhoneVR"),
            Credit(
                "ALVR",
                "alvr-org and polygraphene",
                "Streams SteamVR between PC and phone. Phone PCVR embeds the ALVR v20.8.0 client " +
                    "library and works with the stock ALVR Streamer on the PC.",
                "MIT",
                "https://github.com/alvr-org/ALVR"),
            Credit(
                "Cardboard SDK",
                "Google, built from the nift4/cardboard fork",
                "Lens distortion, gyroscope head tracking and headset QR profiles.",
                "Apache-2.0",
                "https://github.com/googlevr/cardboard"),
            Credit(
                "ARCore SDK",
                "Google",
                "Camera-based 6DoF position tracking.",
                "Apache-2.0",
                "https://github.com/google-ar/arcore-android-sdk"),
            Credit(
                "MediaPipe Hand Landmarker",
                "Google",
                "Hand landmark detection (Tasks Vision library and model) for experimental hand " +
                    "tracking.",
                "Apache-2.0",
                "https://github.com/google-ai-edge/mediapipe"),
            Credit(
                "zxing-cpp",
                "zxing-cpp contributors",
                "QR code decoding for headset profiles.",
                "Apache-2.0",
                "https://github.com/zxing-cpp/zxing-cpp"),
            Credit(
                "Protocol Buffers (Java Lite)",
                "Google",
                "Headset profile format.",
                "BSD-3-Clause",
                "https://github.com/protocolbuffers/protobuf"),
            Credit(
                "AndroidX, CameraX, Material Components, Kotlin",
                "Google and JetBrains",
                "App framework, camera and user interface.",
                "Apache-2.0",
                "https://developer.android.com/jetpack/androidx"),
            Credit(
                "Rust crates used by ALVR",
                "Their respective authors",
                "Networking, serialisation and more inside the ALVR client library. The full " +
                    "list is in ALVR's Cargo.lock.",
                "MIT / Apache-2.0 and others",
                "https://github.com/alvr-org/ALVR/blob/v20.8.0/Cargo.lock"),
        )

    /** Licence button label → asset file. */
    private val licenses =
        listOf(
            "GNU GPL v3.0 (Phone PCVR, PhoneVR)" to "licenses/GPL-3.0.txt",
            "MIT (ALVR)" to "licenses/ALVR-MIT.txt",
            "Apache License 2.0" to "licenses/Apache-2.0.txt",
            "BSD 3-Clause (Protocol Buffers)" to "licenses/protobuf-BSD-3-Clause.txt",
        )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_about)

        findViewById<MaterialButton>(R.id.back).setOnClickListener { finish() }
        findViewById<TextView>(R.id.version).text =
            getString(R.string.version_fmt, BuildConfig.VERSION_NAME)

        val list = findViewById<LinearLayout>(R.id.credits)
        val inflater = LayoutInflater.from(this)
        for (c in credits) {
            val row = inflater.inflate(R.layout.item_credit, list, false)
            row.findViewById<TextView>(R.id.name).text = c.name
            row.findViewById<TextView>(R.id.license).text = c.license
            row.findViewById<TextView>(R.id.by).text = c.by
            row.findViewById<TextView>(R.id.what).text = c.what
            row.findViewById<TextView>(R.id.url).text = c.url.removePrefix("https://")
            row.setOnClickListener { openUrl(c.url) }
            list.addView(row)
        }

        val buttons = findViewById<LinearLayout>(R.id.license_buttons)
        for ((label, asset) in licenses) {
            val b =
                MaterialButton(
                    this, null, com.google.android.material.R.attr.materialButtonOutlinedStyle)
            b.text = label
            b.isAllCaps = false
            b.setOnClickListener { showLicense(label, asset) }
            buttons.addView(b)
        }
        val source =
            MaterialButton(
                this, null, com.google.android.material.R.attr.materialButtonOutlinedStyle)
        source.text = getString(R.string.source_code)
        source.isAllCaps = false
        source.setOnClickListener { openUrl(SOURCE_URL) }
        buttons.addView(source)
    }

    companion object {
        const val SOURCE_URL = "https://github.com/Lennoxx360/phone-pcvr"
    }

    private fun openUrl(url: String) {
        try {
            startActivity(Intent(Intent.ACTION_VIEW, Uri.parse(url)))
        } catch (e: Exception) {
            // No browser installed; nothing to do.
        }
    }

    private fun showLicense(title: String, asset: String) {
        val text = assets.open(asset).bufferedReader().use { it.readText() }
        val pad = (16 * resources.displayMetrics.density).toInt()
        val tv =
            TextView(this).apply {
                setText(text)
                setTextIsSelectable(true)
                setTextSize(TypedValue.COMPLEX_UNIT_SP, 12f)
                typeface = android.graphics.Typeface.MONOSPACE
                setPadding(pad, pad, pad, pad)
            }
        MaterialAlertDialogBuilder(this)
            .setTitle(title)
            .setView(ScrollView(this).apply { addView(tv) })
            .setPositiveButton(android.R.string.ok, null)
            .show()
    }
}
