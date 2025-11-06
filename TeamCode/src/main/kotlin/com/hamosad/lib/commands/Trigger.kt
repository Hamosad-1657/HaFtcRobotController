package com.hamosad.lib.commands

/** Contains a condition, and allows you to bind commands in certain ways to it. */
class Trigger(condition: () -> Boolean) {
    init {
        CommandScheduler.registerTrigger(this)
    }

    private val bindings: Map<() -> Boolean, Command> = mapOf()

    fun getCommandsToRun(): List<Command> {
        val list = mutableListOf<Command>()

        for (binding in bindings) {
            if (binding.key()) {
                list.add(binding.value)
            }
        }

        return list.toList()
    }

    fun getCommandsToNotRun(): List<Command> {
        val list = mutableListOf<Command>()

        for (binding in bindings) {
            if (!binding.key()) {
                list.add(binding.value)
            }
        }

        return list.toList()
    }
    // TODO: ADD BODY
}