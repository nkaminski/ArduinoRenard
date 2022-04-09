# ArduinoRenard - An Arduino library to receive and decode the Renard lighting protocol

## Features
* Receiving / decoding the Renard protocol per the definition at https://www.doityourselfchristmas.com/wiki/index.php?title=Renard#Protocol
    *  Tested sources: OLA, xLights, FPP. 

* Protocol extensions using an address/command byte of 0x7a
    * This can be used for implementing support for features such as remote configuration / addressing

## Limitations
* Receive only
* No forwarding support

## Compile-time options
* RENARD_LARGE_ADDRESS: Enables enumeration of 65535 instead of 255 channels. Increases RAM usage.
* RENARD_DYNAMIC_BUFFERS: Enables support for capturing/processing over 32 channels by dynamilcally allocating a channel data buffer.

## API
Initializes an instance of this library, configuring the provided serial port to the specified baudrate.
Channels must be less than or equal to 32 unless built with RENARD_DYNAMIC_BUFFERS
```c
    void begin(renard_addr_t channels, HardwareSerial *hwSerial, uint32_t baud);
```

Gets the number of channels that are being captured/processed.
```c
    renard_addr_t getChannelCount();
```
Gets/sets the offset from the first data byte that data begins being captured at.
Data at byte locations starting from this value up until (offset + channel_count) are captured. Setting the channel offset clears buffered data.
```c
    renard_addr_t getChannelOffset();
    void setChannelOffset(renard_addr_t count);
```

Gets/sets the idle timeout and idle state.
If a valid start and command byte has not been received for longer than the idle timeout (specified in milliseconds),
isIdle will return true.
```c
    unsigned long getIdleTimeout();
    void setIdleTimeout(unsigned long timeout);
    bool isIdle();
```

Reads a byte from the channel data buffer. If implicit_process is set to true or not provided,
all incoming data in the serial receive queue is processed prior to the read occurring.
```c
    byte read(renard_addr_t address, bool implicit_process=true);
```

Processes all pending data in the serial receive buffer. Must be called frequently in order to avoid data loss.
Returns true if any bytes were processed.
```c
    bool process();
```

Checks if a 0x7a extension/special command is awaiting processing If implicit_process is set to true or not provided,
incoming data up to the 1st special command in the serial receive queue is processed prior to returning.

Calling readSpecialData() is required to clear the flag which reports that a special command awaits processing.
```c
    bool specialAvailable(bool implicit_process=true);
    byte readSpecialOpcode();
    byte readSpecialData();
```

Terminates operation of the library class instance, releasing any allocated resources.
```c
    void end();
```