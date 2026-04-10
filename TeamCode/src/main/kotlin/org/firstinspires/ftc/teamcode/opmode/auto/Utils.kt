package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.async
import kotlinx.coroutines.delay

@Suppress("KotlinUnreachableCode")
fun CoroutineScope.getFiducialId(limelight: Limelight3A): Deferred<Int> = async {
    while(true) {
        limelight.latestResult.takeIf { it.isValid }?.fiducialResults?.single()?.fiducialId?.let { return@async it }
        delay(50L)
    }
    // unreachable, but required to satisfy the compiler since it doesn't know that the while loop will always return
    0
}