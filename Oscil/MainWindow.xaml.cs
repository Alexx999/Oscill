using System;
using System.Collections.Generic;
using System.ComponentModel;
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
            var tgtSerial = "201501082132283";
            var devices = SiUsbDeviceProperties.GetAll();
            var tgtDevice = devices.FirstOrDefault(d => d.SerialNumber == tgtSerial);

            if (tgtDevice == null)
            {
                Debug.WriteLine("Oscill not found");
                return;
            }

            _device = new OscillDevice(tgtDevice.Id);
            var connected = await _device.ConnectAsync().ConfigureAwait(false);
            if (connected)
            {
                Debug.WriteLine("Oscill connected");
            }
            else
            {
                Debug.WriteLine("Oscill not connected");
                return;
            }
            await _device.SetSpeedAsync(921600).ConfigureAwait(false);
            var model = await _device.GetDeviceInfoAsync(DeviceInfoField.Model).ConfigureAwait(false);
            var hard = await _device.GetDeviceInfoAsync(DeviceInfoField.Hard).ConfigureAwait(false);
            var soft = await _device.GetDeviceInfoAsync(DeviceInfoField.Soft).ConfigureAwait(false);
            var serial = await _device.GetDeviceInfoAsync(DeviceInfoField.Serial).ConfigureAwait(false);
            var ds = await _device.SetParametherAsync(ParametherField.DelayedSweep, 0).ConfigureAwait(false);
            var asw = await _device.SetParametherAsync(ParametherField.AlignSweep, 38).ConfigureAwait(false);
            var sample = await _device.SetParametherAsync(ParametherField.SampleMethod, (byte)0).ConfigureAwait(false);
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            _device.SendConnect();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            _device?.Dispose();
            base.OnClosing(e);
        }
    }
}
