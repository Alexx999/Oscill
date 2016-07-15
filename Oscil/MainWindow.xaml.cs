using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using SiUSBXp;

namespace Oscil
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private OscillDevice _device;

        public MainWindow()
        {
            InitializeComponent();
        }

        private async void button_Click(object sender, RoutedEventArgs e)
        {
            var res = SiUsbDevice.NumDevices;
            var desc = SiUsbDevice.GetProductString(0, ProductProperty.Description);
            
            _device = new OscillDevice(0);

            /*
            var res = SiUsbDevice.NumDevices;
            var desc = SiUsbDevice.GetProductString(0, ProductProperty.Description);
            SiUsbDevice.SetTimeouts(1000, 1000);
            var device = SiUsbDevice.Open(0);
            device.SetBaudRate(9600);
            device.SetLineControl(StopBits.One, Parity.None, 8);
            device.SetFlowControl(RxPinOption.StatusInput, TxPinOption.HeldInactive, TxPinOption.HeldActive, RxPinOption.StatusInput, RxPinOption.StatusInput, false);
            var buffer = new byte[] {0x80, 0x00, 0x07, 0x10, 0x00, 0x10, 0x00};

            var rb = new byte[16000];
            device.Write(buffer, 0, buffer.Length);
            await Task.Delay(10);
            var read = device.Read(rb, 0, 16000);
            device.Close();
            Debug.WriteLine("Success");*/
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            _device.SendConnect();
        }
    }
}
