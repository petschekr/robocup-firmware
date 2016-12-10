#pragma once

#include <string>
#include <vector>

/**
 * typedef for a command function's argument(s)
 */
using cmd_args_t = const std::vector<std::string>;

/**
 * Max number of command aliases.
 */
constexpr uint8_t MAX_ALIASES = 5;

/**
 * max command args safety check. Args are now vector based upon creation, so
 * this can be changed in size safely.
 */
constexpr uint8_t MAX_COMMAND_ARGS = 16;

// forward declaration of tasks
void Task_SerialConsole(const void* args);

#ifndef NDEBUG

#include "motors.hpp"
#include "robot-devices.hpp"

#include <algorithm>
#include <array>
#include <memory>

// forward declaration of tasks
void Task_SerialConsole(const void* args);

/**
 * command structure
 */
struct command_t {
    /**
     * aliases
     */
    const std::array<std::string, MAX_ALIASES> aliases;

    /**
     * iterative flag. Should the command be executed iteratively (in the
     * main loop) until the break signal is sent?
     */
    const bool is_iterative;

    /**
     * command handler function pointer
     */
    int (*handler)(cmd_args_t& args);

    /**
     * command description. Used by help
     */
    const std::string description;

    /**
     * usage description. Used by help
     */
    const std::string usage;
};

/*
 * Command functions.
 */
#endif
void execute_line(char*);
void execute_iterative_command();
#ifndef NDEBUG

void show_invalid_args(cmd_args_t&);
void show_invalid_args(const std::string&);

/*
 * Command definitions. Some command functions have circular definitions.
 *
 * Alphabetical order please.
 */
int cmd_alias(cmd_args_t&);
int cmd_baudrate(cmd_args_t&);
int cmd_console_clear(cmd_args_t&);
int cmd_console_echo(cmd_args_t&);
int cmd_console_exit(cmd_args_t&);
int cmd_console_hostname(cmd_args_t&);
int cmd_console_user(cmd_args_t&);
int cmd_help(cmd_args_t&);
int cmd_help_detail(cmd_args_t&);
int cmd_serial_ping(cmd_args_t&);
int cmd_info(cmd_args_t&);
int cmd_interface_check_conn(cmd_args_t&);
int cmd_interface_disconnect(cmd_args_t&);
int cmd_interface_reset(cmd_args_t&);
int cmd_kicker(cmd_args_t&);
int cmd_led(cmd_args_t&);
int cmd_log_level(cmd_args_t&);
int cmd_ls(cmd_args_t&);
int cmd_ping(cmd_args_t&);
int cmd_ps(cmd_args_t& args = {""});
#endif
int cmd_heapfill(cmd_args_t& args = {""});
#ifndef NDEBUG
int cmd_radio(cmd_args_t&);
int cmd_ping(cmd_args_t&);
int cmd_pong(cmd_args_t&);
int cmd_rpc(cmd_args_t&);
int cmd_imu(cmd_args_t&);

#endif
