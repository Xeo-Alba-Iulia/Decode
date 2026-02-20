package org.firstinspires.ftc.teamcode.elevator

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@SingleIn(OpModeScope::class)
@Inject
class Elevator(private val servoList: List<Servo>) : OpModeObserver {
    var position: Double
        get() = servoList.first().position
        set(value) = servoList.forEach { it.position = value }

    init {
        goDown()
    }

    fun goDown() {
        position = 0.5
    }

    fun goUp() {
        position = 0.45
    }
}