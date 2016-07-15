using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;
using MiscUtil.Conversion;
using MiscUtil.IO;
using SiUSBXp;

namespace Oscil
{
    public class OscillDevice : IDisposable
    {
        private SiUsbDevice _device;
        private Task _rxTask;
        private Task _txTask;
        private const int _bufferLength = 4096;
        private readonly byte[] _buffer = new byte[_bufferLength];
        private int _position;
        private readonly BufferBlock<Packet> _sendBuffer = new BufferBlock<Packet>();
        private bool _disposed;
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();

        public OscillDevice(SiUsbDevice device)
        {
            SiUsbDevice.SetTimeouts(1000, 1000);
            _device = device;

            StartLoops();
            Connect();
        }

        private void Connect()
        {
            _device.SetBaudRate(9600);
            _device.SetLineControl(StopBits.One, Parity.None, 8);
            _device.SetFlowControl(RxPinOption.StatusInput, TxPinOption.HeldInactive, TxPinOption.HeldActive, RxPinOption.StatusInput, RxPinOption.StatusInput, false);

            SendPacket(new Packet(Opcode.Connect, new byte[] {16,0,16,0}));
        }

        private void SendPacket(Packet packet)
        {
            _sendBuffer.Post(packet);
        }

        public OscillDevice(uint deviceIndex) : this(SiUsbDevice.Open(deviceIndex))
        {
        }

        private void StartLoops()
        {
            _rxTask = Task.Run(ReadAndProcessLoopAsync);
            _txTask = Task.Run(WritePacketAsync);
        }

        private async Task WritePacketAsync()
        {
            while (!_disposed)
            {
                try
                {
                    var nextPacket = await _sendBuffer.ReceiveAsync(_cts.Token).ConfigureAwait(false);
                    await WritePacketToDeviceAsync(nextPacket, _cts.Token).ConfigureAwait(false);
                }
                catch (Exception)
                {
                }
            }
        }

        private Task WritePacketToDeviceAsync(Packet nextPacket, CancellationToken token)
        {
            var memoryStream = new MemoryStream();
            var writer = new EndianBinaryWriter(EndianBitConverter.Big, memoryStream);
            writer.Write((byte)nextPacket.Opcode);
            writer.Write((short)(nextPacket.Data.Length + 3));
            writer.Write(nextPacket.Data, 0, nextPacket.Data.Length);
            writer.Flush();

            var bytes = memoryStream.ToArray();

            return _device.WriteAsync(bytes, 0, bytes.Length, token);
        }

        private async Task ReadAndProcessLoopAsync()
        {
            while (!_disposed)
            {
                try
                {
                    await ReadAndProcessAsync(_cts.Token).ConfigureAwait(false);
                }
                catch (Exception)
                {
                }
            }
        }

        private async Task ReadAndProcessAsync(CancellationToken token)
        {
            var read = await _device.ReadAsync(_buffer, _position, _bufferLength - _position, token).ConfigureAwait(false);
            _position += read;

            TryReadPacket();
        }

        private void TryReadPacket()
        {
            if(_position < 3) return;
            var type = (Opcode)_buffer[0];
            var converter = new BigEndianBitConverter();
            var len = converter.ToUInt16(_buffer, 1);
            if(_position < len) return;
            var data = new byte[len - 3];
            Buffer.BlockCopy(_buffer, 3, data, 0, data.Length);

            var packet = new Packet(type, data);
            Buffer.BlockCopy(_buffer, _position, _buffer, 0, _bufferLength - _position);
        }

        public virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                _cts.Cancel();
                _cts.Dispose();
                _rxTask.Dispose();
                _txTask.Dispose();
            }
            _device.Close();
            _device.Dispose();
        }

        public void Dispose()
        {
            if(_disposed) return;
            _disposed = true;

            GC.SuppressFinalize(this);
            Dispose(true);
        }

        ~OscillDevice()
        {
            Dispose(false);
        }

        public void SendConnect()
        {
            SendPacket(new Packet(Opcode.Connect, new byte[] { }));
        }
    }

    public enum Opcode : byte
    {
        ChangeSpeed = 0x91,
        CommandF = 0x84,
        CommandN = 0x04,
        Connect = 0x80,
        DataGetF = 0x83,
        DataGetN = 0x03,
        DataPutF = 0x82,
        DataPutN = 0x02,
        Disconnect = 0x81,
        ResendLastResp = 0x92,
        Abort = 0xff,
        AcceptSpeed = 0x0f,
        Continue = 0x90,
        Error = 0xd0,
        Forbidden = 0xc3,
        IntServErr = 0xd0,
        LenReq = 0xcb,
        NotImpl = 0xd1,
        Success = 0xa0
    }

    public class Packet
    {
        public Opcode Opcode;
        public byte[] Data;

        public Packet(Opcode opcode, byte[] bytes)
        {
            Opcode = opcode;
            Data = bytes;
        }
    }
}
