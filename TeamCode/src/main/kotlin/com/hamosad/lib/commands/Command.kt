package com.hamosad.lib.commands

import com.hamosad.lib.math.Seconds

abstract class Command {
    abstract val requirements: List<Subsystem>

    abstract fun initialize()
    abstract fun execute()
    abstract fun onEnd(wasInterrupted: Boolean)
    abstract fun isFinished(): Boolean
}

class SequentialCommandGroup(vararg commands: Command): Command() {
    val commandsList = commands.toList()

    var combinedRequirements: List<Subsystem> = listOf()
    init {
        for (i in 0..commandsList.lastIndex) {
            combinedRequirements = combinedRequirements.union(commandsList[i].requirements).toList()
        }
    }
    override val requirements: List<Subsystem>
        get() = combinedRequirements

    override fun initialize() {
        if (commandsList.lastIndex != -1) commandsList[0].initialize()
    }

    var currentIndex = 0
    override fun execute() {
        if (commandsList.lastIndex != -1) {
            val command = commandsList[currentIndex]

            command.execute()
            if (command.isFinished() && currentIndex < commandsList.lastIndex) {
                currentIndex++
                command.onEnd(false)
                commandsList[currentIndex].initialize()
            }
        }
    }

    override fun isFinished(): Boolean {
        if (currentIndex == commandsList.lastIndex) {
            return commandsList[currentIndex].isFinished()
        }
        return false
    }

    override fun onEnd(wasInterrupted: Boolean) {
        if (wasInterrupted) {
            commandsList[currentIndex].onEnd(true)
        }
    }
}

class ParallelCommandGroup(vararg commands: Command): Command() {
    val commandsList = commands.toList()
    val activeCommandsList = commandsList.toMutableList()

    var combinedRequirements: List<Subsystem> = listOf()
    init {
        for (i in 0..commandsList.lastIndex) {
            combinedRequirements = combinedRequirements.union(commandsList[i].requirements).toList()
        }
    }
    override val requirements: List<Subsystem>
        get() = combinedRequirements

    override fun initialize() {
        for (command in commandsList) {
            command.initialize()
        }
    }

    override fun execute() {
        for (command in activeCommandsList) {
            command.execute()
            if (command.isFinished()) {
                command.onEnd(false)
                activeCommandsList.remove(command)
            }
        }
    }

    override fun isFinished(): Boolean {
        return activeCommandsList.isEmpty()
    }

    override fun onEnd(wasInterrupted: Boolean) {
        if (wasInterrupted) {
            for (command in activeCommandsList) {
                command.onEnd(true)
            }
        }
    }
}

class ParallelRaceCommandGroup(vararg commands: Command): Command() {
    val commandsList = commands.toList()

    var combinedRequirements: List<Subsystem> = listOf()
    init {
        for (i in 0..commandsList.lastIndex) {
            combinedRequirements = combinedRequirements.union(commandsList[i].requirements).toList()
        }
    }
    override val requirements: List<Subsystem>
        get() = combinedRequirements

    override fun initialize() {
        for (command in commandsList) {
            command.initialize()
        }
    }

    override fun execute() {
        for (command in commandsList) {
            command.execute()
        }
    }

    override fun isFinished(): Boolean {
        for (command in commandsList) {
            if (command.isFinished()) return true
        }
        return false
    }

    override fun onEnd(wasInterrupted: Boolean) {
        for (command in commandsList) {
            command.onEnd(wasInterrupted)
        }
    }
}

fun Subsystem.runCommand(code: () -> Unit): Command = object: Command() {
    override val requirements: List<Subsystem> = listOf(this@runCommand)
    override fun initialize() {}
    override fun execute() { code() }
    override fun isFinished(): Boolean = false
    override fun onEnd(wasInterrupted: Boolean) {}
}

fun Subsystem.runOnce(code: () -> Unit): Command = object: Command() {
    override val requirements: List<Subsystem> = listOf(this@runOnce)
    override fun initialize() { code() }
    override fun execute() {}
    override fun isFinished(): Boolean = true
    override fun onEnd(wasInterrupted: Boolean) {}
}

fun waitUntilCommand(condition: () -> Boolean): Command = object: Command() {
    override val requirements: List<Subsystem> = listOf()
    override fun initialize() {}
    override fun execute() {}
    override fun isFinished(): Boolean = condition()
    override fun onEnd(wasInterrupted: Boolean) {}
}

fun waitCommand(time: Seconds): Command = object: Command() {
    var startTime: Long = 0L

    override val requirements: List<Subsystem> = listOf()
    override fun initialize() { startTime = System.currentTimeMillis() }
    override fun execute() {}
    override fun isFinished(): Boolean = System.currentTimeMillis() > startTime + (time * 1000).toLong()
    override fun onEnd(wasInterrupted: Boolean) {}
}

infix fun Command.withTimeout(condition: () -> Boolean): Command = ParallelRaceCommandGroup(this, waitUntilCommand(condition))
infix fun Command.withTimeout(time: Seconds): Command = ParallelRaceCommandGroup(this, waitCommand(time))

infix fun Command.andThen(other: Command): SequentialCommandGroup {
    return SequentialCommandGroup(this, other)
}