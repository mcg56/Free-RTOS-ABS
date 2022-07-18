# TivaWare + FreeRTOS Makefile Example

This project is an example of how the TM4C microcontrollers can be programmed
using GCC and OpenOCD as the primary toolchain. This includes the TivaWare and
FreeRTOS libraries also.

This repository has been adapted fron Ben Mitchell's
[original](https://eng-git.canterbury.ac.nz/bmi32/464_freertos_makefile).

## Requirements

This project was originally developed for use in a Linux environment. Some
effort will be needed to port this to a Windows host.

### Linux

On Ubuntu/Debian systems, use your system's package manager:

```
sudo apt update
sudo apt upgrade
sudo apt install gcc-arm-none-eabi gdb-multiarch openocd
```

### macOS

Using [Homebrew](https://brew.sh/):

    brew install openocd git
    brew cask install gcc-arm-embedded

### Windows

The following packages are expected to be on PATH:

* `make`
* `arm-none-eabi-gcc`
* `arm-none-eabi-gdb`
* `openocd`

Note: if your system provides `gdb-multiarch` instead, you will need to edit the
`tiva.mk` script to account for this.

On the Windows machines in the Embedded Systems Laboratory run the following
command in a Command Prompt window to add the toolchain to your path:

    C:\ence461\ence461-path.bat

This will have to be run for every new CMD session you open.

To install the same toolchain on your personal Windows machine, copy
`C:\ence461` to your machine's `C:` drive. (Or some other location - just
remember to change the path of the above command.)

## Basic Usage

Navigate to the any of the apps provided (where a Makefile is present) and the
following commands are available:

    make         # Build the source code into a binary
    make program # Build the program and then load it onto the microcontroller
    make debug   # Launch GDB and connect to the microcontroller
    make openocd # Launch OpenOCD and connect to the microcontroller

Note that the `program` and `debug` commands require that OpenOCD is running in
the background. Usually this is done in another terminal window using the
`openocd` command.

## Coding style

You will note that there are a variety of coding styles present in this project.
TivaWare uses CamelCase, FreeRTOS uses a prefixed CamelCase, and I use
snake_case. I suggest picking whichever you prefer and sticking with it.
CamelCase would be a good choice here as it is similar to the existing code.

According to
[FreeRTOS](https://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html),
the convention used is to prefix variables and functions with their types (or
return types). This is a form of [Hungarian
notation](https://en.wikipedia.org/wiki/Hungarian_notation) which I personally
feel is uneccesary given that our development environments give us this type
information by simply mousing over the variable in question!
