package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.delay
import kotlinx.coroutines.withTimeoutOrNull
import org.firstinspires.ftc.teamcode.toArtefactList
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

suspend inline fun getPatternList(limelight: Limelight3A?, timeout: Duration = 5.seconds) =
    if (limelight?.isConnected != true) emptyList()
    else withTimeoutOrNull(timeout) {
        while (true) {
            delay(50.milliseconds)
            limelight.latestResult.fiducialResults?.singleOrNull()?.fiducialId?.let { return@withTimeoutOrNull it }
        }
        @Suppress("KotlinUnreachableCode") // make compiler happy
        0
    }?.toArtefactList() ?: emptyList()

fun Pose.mirrorAlliance(isMirrored: Boolean): Pose = if (isMirrored) pose.mirror() else pose
