package org.firstinspires.ftc.teamcode.sorter

import org.firstinspires.ftc.teamcode.ArtefactType
import kotlin.math.abs

class CombinedSorter(vararg val sorters: Sorter) : Sorter {
    override fun prepareIntake() = sorters.forEach { it.prepareIntake() }
    override fun intake(type: ArtefactType) = sorters.forEach { it.intake(type) }
    override fun prepareShoot(type: ArtefactType?) = sorters.all { it.prepareShoot(type) }
    override var isLifting = false
    override val size: Int
        get() {
            val firstSize = sorters.first().size
            val firstArtefacts = sorters.first().artefacts
            require(sorters.all { it.size == firstSize }) { "Sorters have different sizes" }
            require(sorters.all { firstArtefacts.contentEquals(it.artefacts) }) { "Sorters have different artefacts" }
            return firstSize
        }
    override val artefacts: Array<ArtefactType?>
        get() {
            val firstArtefacts = sorters.first().artefacts
            require(sorters.all { firstArtefacts.contentEquals(it.artefacts) }) { "Sorters have different artefacts" }
            return firstArtefacts
        }
    override var position: Double
        get() {
            val firstPosition = sorters.first().position
            require(sorters.all { abs(it.position - firstPosition) <= 0.01 }) { "Sorters have different positions" }
            return firstPosition
        }
        set(value) {
            sorters.forEach { it.position = value }
        }
}