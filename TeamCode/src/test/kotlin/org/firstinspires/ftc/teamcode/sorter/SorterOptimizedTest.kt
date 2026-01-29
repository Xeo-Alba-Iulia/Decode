package org.firstinspires.ftc.teamcode.sorter

import dev.zacsweers.metro.createGraph
import kotlinx.coroutines.test.runTest
import org.firstinspires.ftc.teamcode.ArtefactType
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertDoesNotThrow
import kotlin.math.abs

class SorterOptimizedTest {
    private lateinit var opModeGraph: TestOpModeGraph
    private lateinit var sorter: CombinedSorter

    @BeforeEach
    fun setup() {
        opModeGraph = createGraph()
        sorter = CombinedSorter(opModeGraph.sorterOptimized, opModeGraph.sorterWrapped)
    }

    @Test
    fun `Sorter Optimized and Sorter Wrapped behave the same`() = runTest {
        // Implement test logic to compare behaviors of sorterOptimized and sorterWrapped
        // This is a placeholder for actual test implementation
        sorter.prepareIntake()
        assertTrue(sorter.sorters.all { abs(it.position - sorter.sorters.first().position) <= 0.01 })
        sorter.intake(ArtefactType.PURPLE)
        sorter.intake(ArtefactType.GREEN)
        assertDoesNotThrow { sorter.position }
        assertDoesNotThrow { sorter.size }
        sorter.prepareShoot(ArtefactType.GREEN)
        assertDoesNotThrow { sorter.position }
        assertDoesNotThrow { val _ = sorter.size }
    }
}

