using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;

namespace ReadSerialInputFromUSB.Example
{
    public class SerialInformation
    {
        public SerialPort SerialPort { get; set; }
        private string SerialPortName { get; }

        public SerialInformation(string serialPortName)
        {
            SerialPortName = serialPortName;
        }

        public static IEnumerable<string> GetPorts()
        {
            return SerialPort.GetPortNames();            
        }

        public void ReadFromPort()
        {
            // Initialise the serial port on COM4.
            // obviously we would normally parameterise this, but
            // this is for demonstration purposes only.
            this.SerialPort = new SerialPort(SerialPortName)
            {
                BaudRate = 9600,
                Parity = Parity.None,
                StopBits = StopBits.One,
                DataBits = 8,
                Handshake = Handshake.None
            };

            // Subscribe to the DataReceived event.
            this.SerialPort.DataReceived += SerialPortDataReceived;

            // Now open the port.
            this.SerialPort.Open();
        }

        private void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            var serialPort = (SerialPort)sender;

            // Read the data that's in the serial buffer.
            var serialdata = serialPort.ReadExisting();

            // Write to debug output.
            Debug.Write(serialdata);

            Console.Write(serialdata);
        }
    }
}
