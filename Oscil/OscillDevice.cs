using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
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
        private const int BufferLength = 4096;
        private const uint SpeedBase = 1843200;

        private SiUsbDevice _device;
        private byte[] _buffer;
        private readonly BufferBlock<RequestPacket> _sendBuffer = new BufferBlock<RequestPacket>();
        private bool _disposed;
        private CancellationTokenSource _cts;
        private EndianBinaryReader _reader;
        private MemoryStream _ms;
        private bool _connected;
        private int _connectRetryCount;
        private int _maxConnectRetryCount = 1;
        private TaskCompletionSource<bool> _connectingTcs;
        private TaskCompletionSource<uint> _acceptSpeedTcs;
        private readonly ConcurrentDictionary<DeviceInfoField, TaskCompletionSource<string>> _deviceInfoTasks = new ConcurrentDictionary<DeviceInfoField, TaskCompletionSource<string>>();
        private readonly ConcurrentDictionary<ParametherField, TaskCompletionSource<int>> _parametherFieldTasks = new ConcurrentDictionary<ParametherField, TaskCompletionSource<int>>();
        private readonly ConcurrentDictionary<ParametherField, TaskCompletionSource<byte>> _parametherFieldByteTasks = new ConcurrentDictionary<ParametherField, TaskCompletionSource<byte>>();
        private readonly uint[] _supportedSpeeds = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
        private uint _currentSpeed;
        private Task _rxtask;
        private Task _txtask;


        static OscillDevice()
        {
            SiUsbDevice.SetTimeouts(1000, 1000);
        }

        public OscillDevice(SiUsbDevice device)
        {
            _device = device;
            CurrentSpeed = _supportedSpeeds[0];
            _device.SetLineControl(StopBits.One, Parity.None, 8);
            _device.SetFlowControl(RxPinOption.StatusInput, TxPinOption.HeldInactive, TxPinOption.HeldActive, RxPinOption.StatusInput, RxPinOption.StatusInput, false);
            CreateBuffers();
            StartLoops();
        }

        public bool Connected => _connected;

        public bool Connecting => _connectingTcs != null;

        public uint CurrentSpeed
        {
            get { return _currentSpeed; }
            set
            {
                _currentSpeed = value;
                _device.SetBaudRate(_currentSpeed);
            }
        }

        private void CreateBuffers()
        {
            _buffer = new byte[BufferLength];
            _ms = new MemoryStream(_buffer);
            _ms.SetLength(0);
            _reader = new EndianBinaryReader(EndianBitConverter.Big, _ms);
        }

        private void PacketReceived(ResponsePacket packet)
        {
            Debug.WriteLine($"Oscill: received {packet.Opcode} packet");
            PacketReceived((dynamic)packet);
        }

        private void PacketReceived(ConnectSuccessPacket packet)
        {
            if (Connecting)
            {
                _connected = true;
                _connectingTcs.SetResult(true);
            }
        }

        private void PacketReceived(GenericSuccessPacket packet)
        {
            Debug.WriteLine($"Oscill: received unknown success packet with type {packet.Type:X}");
        }

        private void PacketReceived(GetParametherSuccessPacket packet)
        {
            TaskCompletionSource<int> tcs;
            if (_parametherFieldTasks.TryRemove(packet.ParametherField, out tcs))
            {
                tcs.SetResult(packet.Value);
            }
            else
            {
                Debug.WriteLine("Oscill: Got DeviceInfo response without request");
            }
        }

        private void PacketReceived(GetParametherByteSuccessPacket packet)
        {
            TaskCompletionSource<byte> tcs;
            if (_parametherFieldByteTasks.TryRemove(packet.ParametherField, out tcs))
            {
                tcs.SetResult(packet.Value);
            }
            else
            {
                Debug.WriteLine("Oscill: Got DeviceInfo response without request");
            }
        }

        private void PacketReceived(DeviceInfoSuccessPacket packet)
        {
            TaskCompletionSource<string> tcs;
            if (_deviceInfoTasks.TryGetValue(packet.DeviceInfoField, out tcs))
            {
                tcs.SetResult(packet.Data);
            }
            else
            {
                Debug.WriteLine("Oscill: Got DeviceInfo response without request");
            }
        }

        private void PacketReceived(AcceptSpeedPacket packet)
        {
            var speed = SpeedBase / packet.Divisor;
            CurrentSpeed = speed;
            _acceptSpeedTcs.SetResult(speed);
        }

        private void PacketReceived(ErrorPacket packet)
        {
            if (Debugger.IsAttached)
            {
                Debugger.Break();
            }
        }

        public async Task<bool> ConnectAsync()
        {
            if (_connected) return true;
            _connectingTcs = new TaskCompletionSource<bool>();

            EnqueuePacket(new ConnectPacket());

            var task = _connectingTcs.Task;
            var finishedTask = await Task.WhenAny(Task.Delay(100, _cts.Token), task).ConfigureAwait(false);

            _connectingTcs = null;

            if (ReferenceEquals(task, finishedTask)) return true;

            _cts.Cancel();
            
            await Task.WhenAll(_rxtask, _txtask).ConfigureAwait(false);

            return await CheckAndRetryConnectionAsync().ConfigureAwait(false);
        }

        private async Task<bool> CheckAndRetryConnectionAsync()
        {
            if (_connected) return true;


            if (_connectRetryCount == _maxConnectRetryCount)
            {
                if (!CheckSpeed())
                {
                    return false;
                }

                _connectRetryCount = 0;
            }
            else
            {
                _connectRetryCount++;
            }

            CreateBuffers();
            StartLoops();
            return await ConnectAsync().ConfigureAwait(false);
        }

        private bool CheckSpeed()
        {
            var idx = Array.FindIndex(_supportedSpeeds, v => v == _currentSpeed);
            if (idx == _supportedSpeeds.Length - 1) return false;

            CurrentSpeed = _supportedSpeeds[idx + 1];
            Debug.WriteLine($"Oscill: Speed autoincreased to {CurrentSpeed}");
            return true;
        }

        private void EnqueuePacket(RequestPacket packet)
        {
            _sendBuffer.Post(packet);
        }

        public OscillDevice(uint deviceIndex) : this(SiUsbDevice.Open(deviceIndex))
        {
        }

        private void StartLoops()
        {
            _cts = new CancellationTokenSource();
            _rxtask = Task.Run(ReadAndProcessLoopAsync);
            _txtask = Task.Run(WritePacketAsync);
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
                catch (TaskCanceledException)
                {
                    return;
                }
                catch (OperationCanceledException)
                {
                    return;
                }
                catch
                {
                    // ignored
                }
            }
        }

        private Task WritePacketToDeviceAsync(RequestPacket packet, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var memoryStream = new MemoryStream();
            var writer = new EndianBinaryWriter(EndianBitConverter.Big, memoryStream);
            packet.Write(writer);
            writer.Flush();

            var bytes = memoryStream.ToArray();

            Debug.WriteLine($"Oscill: sending {packet.Opcode} packet");
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
                catch (TaskCanceledException)
                {
                    return;
                }
                catch (OperationCanceledException)
                {
                    return;
                }
                catch
                {
                    // ignored
                }
            }
        }

        private async Task ReadAndProcessAsync(CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var pos = _ms.Length;
            _ms.SetLength(BufferLength);
            var read = await _device.ReadAsync(_buffer, (int)pos, (int)(BufferLength - pos), token).ConfigureAwait(false);
            token.ThrowIfCancellationRequested();
            _ms.SetLength(pos + read);
            _ms.Position = 0;

            TryReadPacket();
        }

        private void TryReadPacket()
        {
            if (_ms.Length == 0) return;

            ResponsePacket packet;

            try
            {
                packet = PacketFactory.GetPacket(_reader);
                packet.Read(_reader);
            }
            catch
            {
                return;
            }

            if (packet.Length == 0)
            {
                throw new InvalidOperationException("Packet has wrong length");
            }

            var pos = _ms.Position;

            if (pos != packet.Length)
            {
                throw new InvalidDataException("Packet wasn't fully read");
            }

            Buffer.BlockCopy(_buffer, (int)pos, _buffer, 0, (int)(BufferLength - pos));
            _ms.SetLength(_ms.Length - _ms.Position);

            PacketReceived(packet);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                _cts.Cancel();
                _cts.Dispose();
                _ms.Dispose();
                _reader.Dispose();
                _device.Close();
                _device.Dispose();
                Debug.WriteLine("Oscill Device disposed");
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            GC.SuppressFinalize(this);
            Dispose(true);
        }

        ~OscillDevice()
        {
            Debug.WriteLine("Warning: Oscill device wasn't disposed");
            Dispose(false);
        }

        public void SendConnect()
        {
            EnqueuePacket(new ConnectPacket());
        }

        public void SendStart()
        {
            EnqueuePacket(new StartPacket());
        }

        public Task<byte> SetParametherAsync(ParametherField field, byte value)
        {
            var task = _parametherFieldByteTasks.GetOrAdd(field, f =>
            {
                var tcs = new TaskCompletionSource<byte>();
                EnqueuePacket(new SetParametherBytePacket(field, value));
                return tcs;
            });

            return task.Task;
        }

        public Task<int> SetParametherAsync(ParametherField field, int value)
        {
            var task = _parametherFieldTasks.GetOrAdd(field, f =>
            {
                var tcs = new TaskCompletionSource<int>();
                EnqueuePacket(new SetParametherPacket(field, value));
                return tcs;
            });

            return task.Task;
        }

        public Task<string> GetDeviceInfoAsync(DeviceInfoField field)
        {
            var task = _deviceInfoTasks.GetOrAdd(field, f =>
            {
                var tcs = new TaskCompletionSource<string>();
                EnqueuePacket(new GetDeviceInfoPacket(field));
                return tcs;
            });

            return task.Task;
        }

        public Task<uint> SetSpeedAsync(uint speed)
        {
            if (!_supportedSpeeds.Contains(speed))
            {
                throw new ArgumentException();
            }

            var divisor = SpeedBase / speed;
            var packet = new ChangeSpeedPacket((byte)divisor);

            _acceptSpeedTcs = new TaskCompletionSource<uint>();
            EnqueuePacket(packet);
            return _acceptSpeedTcs.Task;
        }
    }

    internal static class PacketFactory
    {
        public static ResponsePacket GetPacket(EndianBinaryReader reader)
        {
            var opcode = (RespCode) reader.ReadByte();
            switch (opcode)
            {
                case RespCode.Success:
                {
                    return GetSuccessPacket(reader);
                }
                case RespCode.AcceptSpeed:
                {
                    return new AcceptSpeedPacket();
                }
                case RespCode.Error:
                {
                    return new ErrorPacket();
                }
            }

            throw new ArgumentException("Bad value", nameof(opcode));
        }

        private static ResponsePacket GetSuccessPacket(EndianBinaryReader reader)
        {
            var length = reader.ReadUInt16();
            var type = (CommandType) reader.ReadUInt32();
            switch (type)
            {
                case CommandType.Connect:
                {
                    return new ConnectSuccessPacket(length);
                }
                case CommandType.DeviceInfo:
                {
                    return new DeviceInfoSuccessPacket(length);
                }
                case CommandType.GetParametherInt:
                {
                    return new GetParametherSuccessPacket(length);
                }
                case CommandType.GetParametherByte:
                {
                    return new GetParametherByteSuccessPacket(length);
                }
                default:
                {
                    return new GenericSuccessPacket(length, type);
                }
            }
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
        ResendLastResp = 0x92
    }

    public enum RespCode : byte
    {
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

    public enum HeadId : byte
    {
        Name = 0x01,
        Description = 0x05,
        Type = 0x42,
        Time = 0x44,
        Body = 0x48,
        EBody = 0x49,
        Target = 0x46,
        Property = 0x70,
        Register = 0x71,
        Command = 0x72,
        PackSym = 0xB0,
        OneByte = 0xB1,
        Length = 0xC3,
        TwoByte = 0xF0,
        FourByte = 0xF1,
    }

    public enum CommandType : uint
    {
        Connect = 0x10000018,
        DeviceInfo = 0x70000656,
        GetParametherInt =   0x71000554,
        GetParametherByte =  0x71000552,
        GetParametherByte2 = 0x71000541,
    }

    public enum DeviceInfoField : ushort
    {
        Model = 0x4e4d, //VNM
        Hard = 0x4857, //VHW
        Soft = 0x5357, //VSW
        Serial = 0x534E //VSN
    }

    public enum ParametherField : ushort
    {
        DelayedSweep = 0x44F1,
        AlignSweep = 0x43F0,
        SampleMethod = 0x53B1,
        MinNumPasStrob = 0x52B1,
    }

    public class ErrorPacket : ResponsePacket
    {

        public ErrorPacket() : base(RespCode.Error)
        {
        }

        public override void Read(EndianBinaryReader reader)
        {
            Length = reader.ReadUInt16();
        }
    }

    public class StartPacket : RequestPacket
    {
        public byte[] Data { get; set; }

        public StartPacket() : base(Opcode.DataPutF)
        {
            Data = new byte[] { 0x72, 0, 0x04, 43 };
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((short)(Data.Length + 3));
            writer.Write(Data, 0, Data.Length);
        }
    }

    public class GetDeviceInfoPacket : RequestPacket
    {
        public DeviceInfoField Field { get; set; }

        public GetDeviceInfoPacket(DeviceInfoField field) : base(Opcode.DataGetF)
        {
            Field = field;
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((ushort)9);
            writer.Write((uint)CommandType.DeviceInfo);
            writer.Write((ushort)Field);
        }
    }

    public class SetParametherPacket : RequestPacket
    {
        public ParametherField Field { get; set; }
        public int Value { get; set; }

        public SetParametherPacket(ParametherField field, int value) : base(Opcode.DataGetF)
        {
            Field = field;
            Value = value;
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((ushort)0x0D);
            writer.Write((uint)CommandType.GetParametherInt);
            writer.Write((ushort)Field);
            writer.Write(Value);
        }
    }

    public class SetParametherBytePacket : RequestPacket
    {
        public ParametherField Field { get; set; }
        public byte Value { get; set; }

        public SetParametherBytePacket(ParametherField field, byte value) : base(Opcode.DataGetF)
        {
            Field = field;
            Value = value;
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((ushort)0x0D);
            writer.Write((uint)CommandType.GetParametherInt);
            writer.Write((ushort)Field);
            writer.Write(Value);
        }
    }

    public class ConnectPacket : RequestPacket
    {
        public byte[] Data { get; set; }

        public ConnectPacket() : base(Opcode.Connect)
        {
            Data = new byte[] { 0x10, 0, 0x10, 0 };
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((short)(Data.Length + 3));
            writer.Write(Data, 0, Data.Length);
        }
    }

    public class ChangeSpeedPacket : RequestPacket
    {
        public byte Divisor { get; set; }

        public ChangeSpeedPacket(byte divisor) : base(Opcode.ChangeSpeed)
        {
            Divisor = divisor;
        }

        public override void Write(EndianBinaryWriter writer)
        {
            writer.Write((byte)Opcode);
            writer.Write((ushort)4);
            writer.Write(Divisor);
        }
    }

    public abstract class SuccessPacket : ResponsePacket
    {
        public CommandType Type { get; set; }
        
        public SuccessPacket(ushort length, CommandType connect) : base(RespCode.Success)
        {
            Length = length;
            Type = connect;
        }
    }

    public class ConnectSuccessPacket : SuccessPacket
    {
        public ConnectSuccessPacket(ushort length) : base(length, CommandType.Connect)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
        }
    }

    public class GenericSuccessPacket : SuccessPacket
    {
        public byte[] Data { get; set; }

        public GenericSuccessPacket(ushort length, CommandType connect) : base(length, connect)
        {
        }

        public override void Read(EndianBinaryReader reader)
        {
            var dataLen = Length - 7;
            var data = new byte[dataLen];
            reader.Read(data, 0, dataLen);
            Data = data;
        }
    }

    public class DeviceInfoSuccessPacket : SuccessPacket
    {
        public DeviceInfoField DeviceInfoField { get; set; }
        public string Data { get; set; }

        public DeviceInfoSuccessPacket(ushort length) : base(length, CommandType.DeviceInfo)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
            DeviceInfoField = (DeviceInfoField) reader.ReadUInt16();
            var someByte = reader.ReadByte();
            var dataLen = Length - 10;
            var data = reader.ReadBytes(dataLen);
            Data = Encoding.ASCII.GetString(data);
        }
    }

    public class GetParametherSuccessPacket : SuccessPacket
    {
        public ParametherField ParametherField { get; set; }
        public int Value { get; set; }

        public GetParametherSuccessPacket(ushort length) : base(length, CommandType.GetParametherInt)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
            ParametherField = (ParametherField) reader.ReadUInt16();
            Value = reader.ReadInt32();
        }
    }

    public class GetParametherByteSuccessPacket : SuccessPacket
    {
        public ParametherField ParametherField { get; set; }
        public byte Value { get; set; }

        public GetParametherByteSuccessPacket(ushort length) : base(length, CommandType.GetParametherByte)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
            ParametherField = (ParametherField) reader.ReadUInt16();
            Value = reader.ReadByte();
        }
    }

    public class AcceptSpeedPacket : ResponsePacket
    {
        public byte Divisor { get; set; }

        public AcceptSpeedPacket() : base(RespCode.AcceptSpeed) { }

        public override void Read(EndianBinaryReader reader)
        {
            Length = reader.ReadUInt16();
            Divisor = reader.ReadByte();
        }
    }

    public abstract class RequestPacket
    {
        public Opcode Opcode { get; }
        public ushort Length { get; protected set; }

        public RequestPacket(Opcode opcode)
        {
            Opcode = opcode;
        }
        
        public abstract void Write(EndianBinaryWriter writer);
    }

    public abstract class ResponsePacket
    {
        public RespCode Opcode { get; }
        public ushort Length { get; protected set; }

        public ResponsePacket(RespCode opcode)
        {
            Opcode = opcode;
        }

        public abstract void Read(EndianBinaryReader reader);
    }
}
