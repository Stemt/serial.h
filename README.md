# serial.h

A single header serial library for Windows and Linux.

## Simple Example

```c
#define SERIAL_IMPLEMENTATION
#include "../serial.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char** argv){

  SerialPort port = {0};

  const char* port_path = "/dev/ttyUSB0";

  // opens port with default settings, equivalent to
  // ```
  //   serial_open(&port, port_path,
  //      .baud_rate      = 9600,
  //      .char_size      = 8,
  //      .flow_control   = SERIAL_FLOWCTRL_NONE,
  //      .parity         = SERIAL_PARITY_NONE
  //      .stop_bits      = SERIAL_PARITY_NONE
  //      .timeout        = SERIAL_TIMEOUT_INFINITE
  //   )
  // ```
  if(serial_open(&port, port_path) == false){
    fprintf(stderr, "Failed to open port %s: %s\n", port_path, serial_strerror(serial_error(&port)));
    return 1;
  }

  const char* msg = "Hello Device!";
  if(serial_write(&port, msg, strlen(msg)) < 0){
    fprintf(stderr, "Failed to write to port: %s\n", serial_strerror(serial_error(&port)));
    return 1;
  }

  char buf[1024] = {0};
  int n_read = serial_read(&port, buf, sizeof(buf)-1);
  if(n_read < 0){
    fprintf(stderr, "Failed to write to port: %s\n", serial_strerror(serial_error(&port)));
    return 1;
  }else if(n_read == 0){
    fprintf(stderr, "Port read timed out");
    return 1;
  }else{
    fprintf(stderr, "Received: '%s'\n", buf);
  }

  return 0;
}
```

# API

## The `SerialPort` type

A POD type that holds information about the serial port.
An object of this type is produced by `open` and must be passed through to all other functions.

## The `Settings` type

A POD type that holds:

* Baud rate -> `int`
* Character size -> `int` : `5-8`
* Flow control -> `enum class FlowControl` : `NONE` | `SOFTWARE` | `HARDWARE`
* Parity -> `enum class Parity` : `NONE` | `ODD` | `EVEN`
* Stop bits -> `enum class StopBits` : `ONE` | `ONE_POINT_FIVE` | `TWO`

An object of this type should be provided to the `configure` to apply the settings to an open serial port.
The type has sane defaults that apply to most devices.
Usually the only thing that needs changing is the baud rate.

## `open(char* path)`

Provide a valid path to a serial port device and a `SerialPort` object will be returned.

## `configure(SerialPort& sp, Settings settings)`

After opening a port, it should be configured.
Usually the only setting that needs to be changed is the baud rate.

## `set_low_latency(SerialPort& sp, bool low_latency)`

Sets the device latency to 1ms if `low_latency` is true.

## `in_waiting(SerialPort& sp)`

Return the number of bytes waiting to be read off the serial port.

## `clear_input(SerialPort& sp)`

Discard all data that is currently waiting to be read from the port.

## `clear_output(SerialPort& sp)`

Discard all data that is waiting to be written to the port.

## `read(SerialPort& sp, char* buf, int length, int timeout)`

Read bytes from the serial port.

The `buf` should be at least `length` in size.

Timeout:

* `0`: the read will not block but return instantly with the number of bytes read. The buffer will be filled by any bytes read.
* `-1`: the read will block until `length` bytes have been read.
* `> 0`: the read will block until either `length` bytes have been read, or `timeout` milliseconds have passed.

## `write(SerialPort& sp, char* buf, int length, int timeout)`

Write bytes to serial port.

Again, the `buf` should be at least `length` in size.

The rules for `timeout` are the same as with `read`.
