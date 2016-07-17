﻿using System;
using System.Diagnostics;
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
        private const int BufferLength = 4096;
        private byte[] _buffer;
        private readonly BufferBlock<Packet> _sendBuffer = new BufferBlock<Packet>();
        private bool _disposed;
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();
        private EndianBinaryReader _reader;
        private MemoryStream _ms;
        private bool _connected;
        private int _connectRetryCount;
        private int _maxConnectRetryCount = 1;
        private TaskCompletionSource<bool> _connectingTcs;

        static OscillDevice()
        {
            SiUsbDevice.SetTimeouts(1000, 1000);
        }

        public OscillDevice(SiUsbDevice device)
        {
            _device = device;
            _device.SetBaudRate(9600);
            _device.SetLineControl(StopBits.One, Parity.None, 8);
            _device.SetFlowControl(RxPinOption.StatusInput, TxPinOption.HeldInactive, TxPinOption.HeldActive, RxPinOption.StatusInput, RxPinOption.StatusInput, false);
            CreateBuffers();
            StartLoops();
        }

        public bool Connected => _connected;

        public bool Connecting => _connectingTcs != null;

        private void CreateBuffers()
        {
            _buffer = new byte[BufferLength];
            _ms = new MemoryStream(_buffer);
            _ms.SetLength(0);
            _reader = new EndianBinaryReader(EndianBitConverter.Big, _ms);
        }

        private void PacketReceived(Packet packet)
        {
            var successPacket = packet as SuccessPacket;
            if (successPacket != null)
            {
                PacketReceived(successPacket);
            }
        }

        private void PacketReceived(SuccessPacket packet)
        {
            if (Connecting)
            {
                _connected = true;
                _connectingTcs.SetResult(true);
            }
        }

        public async Task<bool> ConnectAsync()
        {
            if (_connected) return true;
            _connectingTcs = new TaskCompletionSource<bool>();

            SendPacket(new ConnectPacket());

            await Task.WhenAny(Task.Delay(1000), _connectingTcs.Task).ConfigureAwait(true);

            _connectingTcs = null;
            return await CheckAndRetryConnectionAsync().ConfigureAwait(false);
        }

        private async Task<bool> CheckAndRetryConnectionAsync()
        {
            if(_connected) return true;

            _connectRetryCount++;

            if (_connectRetryCount == _maxConnectRetryCount)
            {
                return false;
            }

            CreateBuffers();
            return await ConnectAsync().ConfigureAwait(false);
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
            Task.Run(ReadAndProcessLoopAsync);
            Task.Run(WritePacketAsync);
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
                catch
                {
                    // ignored
                }
            }
        }

        private Task WritePacketToDeviceAsync(Packet packet, CancellationToken token)
        {
            var memoryStream = new MemoryStream();
            var writer = new EndianBinaryWriter(EndianBitConverter.Big, memoryStream);
            packet.Write(writer);
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
                catch
                {
                    // ignored
                }
            }
        }

        private async Task ReadAndProcessAsync(CancellationToken token)
        {
            var pos = _ms.Length;
            _ms.SetLength(BufferLength);
            var read = await _device.ReadAsync(_buffer, (int) pos, (int) (BufferLength - pos), token).ConfigureAwait(false);
            _ms.SetLength(pos + read);
            _ms.Position = 0;

            TryReadPacket();
        }

        private void TryReadPacket()
        {
            if(_ms.Length < 1) return;
            var type = (Opcode)_reader.ReadByte();

            Packet packet;

            try
            {
                packet = PacketFactory.GetPacket(type);
                packet.Read(_reader);
            }
            catch
            {
                return;
            }

            var pos = _ms.Position;
            Buffer.BlockCopy(_buffer, (int) pos, _buffer, 0, (int) (BufferLength - pos));
            _ms.SetLength(_ms.Length - _ms.Position);

            PacketReceived(packet);
        }

        public virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                _cts.Cancel();
                _cts.Dispose();
                _ms.Dispose();
                _reader.Dispose();
            }
            _device.Close();
            _device.Dispose();

            Debug.WriteLine("OscillDevice disposed");
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
            SendPacket(new ConnectPacket());
        }
    }

    internal static class PacketFactory
    {
        public static Packet GetPacket(Opcode opcode)
        {
            switch (opcode)
            {
                case Opcode.Success:
                {
                    return new SuccessPacket();
                }
            }

            throw new ArgumentException("Bad value", nameof(opcode));
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

    public class ConnectPacket : Packet
    {
        public byte[] Data { get; set; }

        public ConnectPacket() : base(Opcode.Connect)
        {
            Data = new byte[] {0x10, 0, 0x10, 0};
        }

        public override void Read(EndianBinaryReader reader)
        {
            throw new NotImplementedException();
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((short)(Data.Length + 3));
            writer.Write(Data, 0, Data.Length);
        }
    }

    public class SuccessPacket : Packet
    {
        public byte[] Data { get; set; }

        public SuccessPacket() : base(Opcode.Success) { }
        public override void Read(EndianBinaryReader reader)
        {
            var len = reader.ReadUInt16() - 3;
            var data = new byte[len];
            reader.Read(data, 0, len);
            Data = data;
        }

        public override void Write(EndianBinaryWriter writer)
        {
            throw new NotImplementedException();
        }
    }

    public abstract class Packet
    {
        public Opcode Opcode;

        public Packet(Opcode opcode)
        {
            Opcode = opcode;
        }

        public abstract void Read(EndianBinaryReader reader);
        public abstract void Write(EndianBinaryWriter writer);
    }
}
