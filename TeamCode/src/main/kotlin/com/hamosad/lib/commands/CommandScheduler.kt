package com.hamosad.lib.commands

// TODO: ADD DEFAULT COMMANDS AND FIX ITERATING LIST REMOVE ISSUE
class CommandScheduler(val subsystems: List<Subsystem>) {
    val activeCommands: MutableList<Command> = mutableListOf()

    fun scheduleCommand(command: Command) {
        for (activeCommand in activeCommands) {
            if (activeCommand.requirements.any {it in command.requirements}) {
                activeCommand.onEnd(true)
                activeCommands.remove(activeCommand)
            }
        }
        command.initialize()
        activeCommands.add(command)
    }

    fun execute() {
        for (command in activeCommands) {
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