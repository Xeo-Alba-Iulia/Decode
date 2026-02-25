package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.toArtefactList
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

suspend inline fun getPatternList(limelight: Limelight3A?, timeout: Duration = 5.seconds) = coroutineScope {
    if (limelight?.isConnected != true) return@coroutineScope CompletableDeferred(value = emptyList())
    async {
        withTimeoutOrNull(timeout) {
            while (true) {
                delay(50.milliseconds)
                limelight.latestResult.fiducialResults?.singleOrNull()?.fiducialId?.let { return@withTimeoutOrNull it }
            }
            @Suppress("KotlinUnreachableCode") // make compiler happy
            0
        }?.toArtefactList() ?: emptyList()
    }
}

suspend inline fun outtakeBall(intake: Intake) {
    intake.isOuttake = true
    delay(300.milliseconds)
    intake.isOuttake = false
}