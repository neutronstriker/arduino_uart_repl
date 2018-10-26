# arduino_uart_repl
This is a program which runs on Arduino (Atmega328p variants) and provides the users with UART based command line interface to control various interfaces on the Arduino by either typing or using a script which sends UART commands.


Usage:
        sethX: Set X pin High
        setlX: Set X pin Low
        setiX: Set X pin input
        setpX: Set X pin input Pull-up
        getX: get X pin digital Value
        pwmvX Y: set pwm on X pin of Value Y(0-255)
        pwmfX Y: set pwm frequency divisor on X pin of Value Y[1|8|32|64|128|256|1024]
        agetX: get X pin analog Value
        arefX: set analog reference, arefi:internal, arefx:external, arefv:vcc
        togX: toggle X pin digital output
        streamDX: continuously Stream the state of digital Pin X
        streamAX: continuously Stream the data of analog Pin X
        streamPX: continuously Stream the Value of PORT X, where X can be B, C or D in HEX
        streamS: Stop all streams
        i2cscan: Scan I2C bus for all devices
        i2cping: Checks if a requested I2C slave device is responsive or not
                  format: i2cping -s<slave_addr>
        i2cwrite: Write data to requested I2C slave device
                  format:i2cwrite -s<slave_addr> -n<no. of bytes> -r<register_addr> -h<databyte1,databyte2..> [-tHEX:DEC]
                  the sequence of parameters is not important, max number of bytes can be 8, default is HEX type
                  you can specify DEC if you want.
        i2cread: Read data from requested I2C slave device
                  format:i2cread -s<slave_addr> -n<no. of bytes> -r<register_addr> [-tHEX:DEC]
                  the sequence of parameters is not important, max number of bytes can be 8, default is HEX type
                  you can specify DEC if you want.
        show: Show pin-out on OLED Display
        ate: enable Terminal ECHO
        atde: disable Terminal ECHO
        bspace_e: enable Backspace detection
        bspace_d: disable Backspace detection
        case_e: enable Case sensitivity
        case_d: disable Case sensitivity
        wspr: discard white space characters
        wspk: keep white space characters
        reset: resets the device
        snoope: disables TX of AVR and keeps the RX enabled so that we can read data of another UART device connected
        snoopd: Enables the TX of AVR back ON
        udis: disable Uart connection; can be use to make a pass through connection between Bluetooth and another UART device.
        help: Show this help message

Note: 1.CTRL+C followed by RETURN also stops stream
      2.All three types of stream can be active together and data will be in Comma Separated values

