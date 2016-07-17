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
            var tgtSerial = "201501082132283";
            var devices = SiUsbDeviceProperties.GetAll();
            var tgtDevice = devices.FirstOrDefault(d => d.SerialNumber == tgtSerial);
            
            if(tgtDevice == null) return;

            using (_device = new OscillDevice(tgtDevice.Id))
            {
                var b = await _device.ConnectAsync().ConfigureAwait(false);
            }
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            _device.SendConnect();
        }
    }
}
