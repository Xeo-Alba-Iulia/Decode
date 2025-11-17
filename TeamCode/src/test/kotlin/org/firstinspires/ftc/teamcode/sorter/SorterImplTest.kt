package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.createGraph
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class SorterImplTest : OpMode() {
    lateinit var appGraph: TestOpModeGraph
    lateinit var sorter: SorterImpl
    lateinit var servo: Servo

    @BeforeEach
    override fun init() {
        appGraph = createGraph<TestOpModeGraph>()
        sorter = appGraph.sorterImpl
        servo = FakeSorterServoBinding.fakeServo
    }

    override fun loop() {}

    @Test
    fun intakePosition() {
        assertEquals(servo.position, SorterImpl.INTAKE_POSITIONS[0])
    }
}