package com.hamosad.lib.commands

// TODO: ADD DEFAULT COMMANDS AND FIX ITERATING LIST REMOVE ISSUE
class CommandScheduler(val subsystems: List<Subsystem>) {
    val activeCommands: MutableList<Command> = mutableListOf()

    fun scheduleCommand(command: Command) {
        val iterator = activeCommands.iterator()

        while (iterator.hasNext()) {
            val activeCommand = iterator.next()

            if (activeCommand.requirements.any {it in command.requirements}) {
                activeCommand.onEnd(true)
                activeCommands.remove(activeCommand)
            }
        }
        command.initialize()
        activeCommands.add(command)
    }

    fun execute() {
        val iterator = activeCommands.iterator()
            while (iterator.hasNext()) {
                val command = iterator.next()

                command.execute()
                if (command.isFinished()) {
                    command.onEnd(false)
                    activeCommands.remove(command)
                }
            }

        for (subsystem in subsystems) {
            subsystem.periodic()
        }
    }
}