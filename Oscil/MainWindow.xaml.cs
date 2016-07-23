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
            var model = await _device.GetPropertyAsync("VNM").ConfigureAwait(false);
            var hard = await _device.GetPropertyAsync("VHW").ConfigureAwait(false);
            var soft = await _device.GetPropertyAsync("VSW").ConfigureAwait(false);
            var serial = await _device.GetPropertyAsync("VSN").ConfigureAwait(false);
            var ds = await _device.SetRegisterAsync("TD", 0, SizeId.FourByte).ConfigureAwait(false);
            var asw = await _device.SetRegisterAsync("TC", 38, SizeId.TwoByte).ConfigureAwait(false);
            var sample = await _device.SetRegisterAsync("RS", 0, SizeId.OneByte).ConfigureAwait(false);
            var minNumPassStrob = await _device.SetRegisterAsync("AR", 15, SizeId.OneByte).ConfigureAwait(false);
            var minNumPassAvgPick = await _device.SetRegisterAsync("AP", 0, SizeId.OneByte).ConfigureAwait(false);
            var mCycle = await _device.SetRegisterAsync("MC", 1248, SizeId.TwoByte).ConfigureAwait(false);
            var toSincA = await _device.SetRegisterAsync("TA", 3338675, SizeId.FourByte).ConfigureAwait(false);
            var toSincW = await _device.SetRegisterAsync("TW", 3338675, SizeId.FourByte).ConfigureAwait(false);
            var sync = await _device.SetRegisterAsync("T1", 32, SizeId.OneByte).ConfigureAwait(false);
            var input = await _device.SetRegisterAsync("O1", 0, SizeId.OneByte).ConfigureAwait(false);
            var sampleMode = await _device.SetRegisterAsync("M1", 4, SizeId.OneByte).ConfigureAwait(false);
            var sens = await _device.SetRegisterAsync("V1", 1000, SizeId.TwoByte).ConfigureAwait(false);
            var shift = await _device.SetRegisterAsync("P1", 0, SizeId.TwoByte).ConfigureAwait(false);
            var levelSync = await _device.SetRegisterAsync("S1", 127, SizeId.OneByte).ConfigureAwait(false);
            var typeSync = await _device.SetRegisterAsync("RT", 2, SizeId.OneByte).ConfigureAwait(false);
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            _device.SendStart();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            _device?.Dispose();
            base.OnClosing(e);
        }
    }
}
