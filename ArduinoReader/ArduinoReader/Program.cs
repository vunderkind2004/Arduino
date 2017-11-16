using ReadSerialInputFromUSB.Example;
using System;
using System.Linq;

namespace ArduinoReader
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            try
            {
                StartComunication();
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex);
            }

            Console.ReadLine();
        }

        static void StartComunication()
        {
            var ports = SerialInformation.GetPorts();
            if (ports.Any())
            {
                Console.WriteLine($"Found {ports.Count()} ports: " + ports.Aggregate((x, y) => $"{x} {y}"));
                var port = ports.First();

                Console.WriteLine($"Connecting to {port}");

                var serialInformation = new SerialInformation(port);

                serialInformation.ReadFromPort();

                Console.ReadKey();

                serialInformation.SerialPort.Close();
            }
            else
            {
                Console.WriteLine("No ports was found");
            }
        }
    }
}
