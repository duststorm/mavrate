An example project utilizing the MAVLink library that outputs all ithe transmission rates of received messages.

This program reads from the serial port indicated as its only argument and outputs to standard output.

Requirements
------------
* The generated C code from a MAVLink dialect should be placed in the empty `./mavlink` directory within this project.
  * So if you generated your code into `~/myCode`, you can just copy all of the contents of `~/myCode` into `./mavlink`
* A POSIX-compatible system, as unistd.h is used.
* `gcc`
* `make`

Configuration
-------------
1. Copy whatever MAVLink dialects you want to use into `./mavlink`
2. Modify the `MAVLINK_DIALECT` within `Makefile` to refer to the dialect you want to use.

Usage
-----
::

    $ ./mavrate /dev/ttyUSB0
