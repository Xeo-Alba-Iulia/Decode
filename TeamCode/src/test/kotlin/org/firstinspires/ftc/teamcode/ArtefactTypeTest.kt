package org.firstinspires.ftc.teamcode

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows

class ArtefactTypeTest {
    @Test fun `toArtefactList returns the correct list for valid IDs`() {
        assert(21.toArtefactList() == listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE))
        assert(22.toArtefactList() == listOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE))
        assert(23.toArtefactList() == listOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN))
    }

    @Test fun `toArtefactList throws an exception for invalid IDs`() {
        val invalidIDs = listOf(0, 20, 24, 100)
        for (id in invalidIDs)
            assertThrows<IllegalStateException>("Expected exception for ID: $id") { val _ = id.toArtefactList() }
    }
}