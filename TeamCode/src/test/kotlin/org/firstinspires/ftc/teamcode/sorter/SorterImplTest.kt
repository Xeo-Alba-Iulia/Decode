package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.createGraph
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

class SorterImplTest {
    lateinit var appGraph: TestOpModeGraph
    lateinit var sorter: SorterImpl
    lateinit var servo: Servo

    @BeforeEach
    fun setup() {
        appGraph = createGraph<TestOpModeGraph>()
        sorter = appGraph.sorterImpl
        servo = FakeSorterServoBinding.sorterServo
    }

    @Test
    fun intake() {
        assertEquals(SorterImpl.INTAKE_POSITIONS[0], servo.position)
        assertEquals(
            SorterImpl.INTAKE_POSITIONS[1],
            sorter.intake(ArtefactType.PURPLE).run { servo.position }
        )
        assertEquals(
            SorterImpl.INTAKE_POSITIONS[2],
            sorter.intake(ArtefactType.PURPLE).run { servo.position }
        )
        assertTrue(sorter.run {
            intake(ArtefactType.GREEN)
            isFull
        })
        assertThrows<IllegalArgumentException>("Wrong exception on full shooter") { sorter.intake(ArtefactType.PURPLE) }
        assertEquals("SorterImpl(artefacts = [PURPLE, PURPLE, GREEN])", sorter.toString())
    }
}