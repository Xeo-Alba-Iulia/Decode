package org.firstinspires.ftc.teamcode.intake

import com.qualcomm.robotcore.hardware.NormalizedRGBA
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.transform

operator fun NormalizedRGBA.component1() = alpha
operator fun NormalizedRGBA.component2() = red
operator fun NormalizedRGBA.component3() = blue
operator fun NormalizedRGBA.component4() = green

fun <T> Flow<T>.zipWithNext(): Flow<Pair<T, T>> {
    var previous: T? = null
    var isFirst = true
    return transform {
        if (isFirst) {
            isFirst = false
        } else {
            @Suppress("UNCHECKED_CAST")
            emit((previous as T) to it)
        }
        previous = it
    }
}
