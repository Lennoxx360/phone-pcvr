/* (C)2026 */
package io.github.lennoxx360.phonepcvr

import android.app.Application
import android.content.Context
import android.content.Intent
import android.os.Build
import androidx.core.content.FileProvider
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Local-only crash log. Uncaught Java/Kotlin exceptions are written to
 * Android/data/<package>/files/crash/last_crash.txt. Nothing is uploaded; the user can share the
 * file from the home screen if they want to.
 */
object CrashLog {
    private const val FILE = "last_crash.txt"

    fun install(context: Context) {
        val app = context.applicationContext
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, error ->
            try {
                write(app, thread, error)
            } catch (ignored: Throwable) {}
            previous?.uncaughtException(thread, error)
        }
    }

    private fun dir(context: Context): File =
        File(context.getExternalFilesDir(null) ?: context.filesDir, "crash")

    fun lastCrash(context: Context): File? = File(dir(context), FILE).takeIf { it.exists() }

    private fun write(context: Context, thread: Thread, error: Throwable) {
        val trace = StringWriter().also { error.printStackTrace(PrintWriter(it)) }.toString()
        val time = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date())
        val text = buildString {
            appendLine("Phone PCVR ${BuildConfig.VERSION_NAME} (${BuildConfig.VERSION_CODE})")
            appendLine("Time: $time")
            appendLine(
                "Device: ${Build.MANUFACTURER} ${Build.MODEL}, Android ${Build.VERSION.RELEASE}")
            appendLine("Thread: ${thread.name}")
            appendLine()
            append(trace)
        }
        val d = dir(context)
        d.mkdirs()
        File(d, FILE).writeText(text)
    }

    fun share(context: Context, file: File) {
        val uri = FileProvider.getUriForFile(context, "${context.packageName}.provider", file)
        val send =
            Intent(Intent.ACTION_SEND)
                .setType("text/plain")
                .putExtra(Intent.EXTRA_STREAM, uri)
                .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        context.startActivity(Intent.createChooser(send, null))
    }
}

class PcvrApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        CrashLog.install(this)
    }
}
