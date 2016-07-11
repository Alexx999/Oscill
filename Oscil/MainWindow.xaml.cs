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
        public MainWindow()
        {
            InitializeComponent();
        }

        private async void button_Click(object sender, RoutedEventArgs e)
        {
            var res = SiUsbDevice.NumDevices;
            var desc = SiUsbDevice.GetProductString(0, ProductProperty.Pid);
            SiUsbDevice.SetTimeouts(1000, 1000);
            var device = SiUsbDevice.Open(0);
            device.SetBaudRate(9600);
            device.SetLineControl(StopBits.One, Parity.None, 8);
            device.SetFlowControl(RxPinOption.StatusInput, TxPinOption.HeldInactive, TxPinOption.HeldActive, RxPinOption.StatusInput, RxPinOption.StatusInput, false);
            var buffer = new byte[] {0x80, 0x00, 0x07, 0x10, 0x00, 0x10, 0x00};
            //device.Write(buffer, 0, buffer.Length);
            /*RxQueueState state;
            do
            {
                state = device.CheckRxQueue();
                await Task.Delay(100);
            } while (state.QueueStatus != RxQueueStatus.Ready);*/
            var rb = new byte[16000];
            /*var read = device.Read(rb, 0, 512);
            var b = 10;*/
            await device.WriteAsync(buffer, 0, buffer.Length);
            var read = await device.ReadAsync(rb, 0, 16000);
            device.Close();
            Debug.WriteLine("Success");
        }
    }
}
