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
        private readonly ConcurrentDictionary<string, TaskCompletionSource<string>> _propertyTasks = new ConcurrentDictionary<string, TaskCompletionSource<string>>();
        private readonly ConcurrentDictionary<string, TaskCompletionSource<int>> _parametherFieldTasks = new ConcurrentDictionary<string, TaskCompletionSource<int>>();
        private readonly uint[] _supportedSpeeds = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
        private uint _currentSpeed;
        private Task _rxtask;
        private Task _txtask;
        private ushort _maxPacketSize;


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
                _maxPacketSize = packet.MaxPacketSize;
                _connected = true;
                _connectingTcs.SetResult(true);
            }
        }

        private void PacketReceived(GenericSuccessPacket packet)
        {
            Debug.WriteLine($"Oscill: received unknown success packet with type {packet.Type:X}");
        }

        private void PacketReceived(SetRegisterSuccessPacket packet)
        {
            TaskCompletionSource<int> tcs;
            if (_parametherFieldTasks.TryRemove(packet.RegisterName, out tcs))
            {
                tcs.SetResult(packet.Value);
            }
            else
            {
                Debug.WriteLine("Oscill: Got DeviceInfo response without request");
            }
        }

        private void PacketReceived(PropertySuccessPacket packet)
        {
            TaskCompletionSource<string> tcs;
            if (_propertyTasks.TryGetValue(packet.PropertyName, out tcs))
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

            EnqueuePacket(new ConnectPacket(BufferLength));

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

            var bytes = packet.GetBytes();

            Debug.WriteLine($"Oscill: sending {packet.Opcode} packet of type {packet.GetType()}");
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
                packet = PacketHelper.GetPacket(_reader);
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

        public void SendStart()
        {
            EnqueuePacket(new StartPacket());
        }

        public Task<int> SetRegisterAsync(string registerName, int value, SizeId sizeId)
        {
            var task = _parametherFieldTasks.GetOrAdd(registerName, f =>
            {
                var tcs = new TaskCompletionSource<int>();
                EnqueuePacket(new SetRegisterPacket(registerName, value, sizeId));
                return tcs;
            });

            return task.Task;
        }

        public Task<string> GetPropertyAsync(string propertyName)
        {
            var task = _propertyTasks.GetOrAdd(propertyName, f =>
            {
                var tcs = new TaskCompletionSource<string>();
                EnqueuePacket(new GetPropertyPacket(propertyName));
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

    internal static class PacketHelper
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
            var type = (HeadId) reader.ReadByte();
            switch (type)
            {
                case HeadId.Connect:
                {
                    return new ConnectSuccessPacket(length);
                }
                case HeadId.Property:
                {
                    return new PropertySuccessPacket(length);
                }
                case HeadId.Register:
                {
                    return new SetRegisterSuccessPacket(length);
                }
                default:
                {
                    return new GenericSuccessPacket(length, type);
                }
            }
        }

        private static int GetByteCount(SizeId headId)
        {
            switch (headId)
            {
                case SizeId.OneByte:
                {
                    return 1;
                }
                case SizeId.TwoByte:
                {
                    return 2;
                }
                case SizeId.FourByte:
                {
                    return 4;
                }
                default:
                {
                    throw new ArgumentOutOfRangeException(nameof(headId), "Head ID is not an encoded length");
                }
            }
        }

        public static string ReadHeaderString(this EndianBinaryReader reader)
        {
            var nameLen = reader.ReadUInt16() - 3;
            var nameBytes = reader.ReadBytes(nameLen);
            return Encoding.ASCII.GetString(nameBytes);
        }

        public static void WriteHeaderString(this EndianBinaryWriter writer, string str)
        {
            writer.Write((ushort)(str.Length + 3));
            writer.Write(Encoding.ASCII.GetBytes(str));
        }

        public static byte[] ReadHeaderDataBytes(this EndianBinaryReader reader)
        {
            var dataLenId = (SizeId)reader.ReadByte();
            var dataLen = GetByteCount(dataLenId);
            return reader.ReadBytes(dataLen);
        }

        public static void WriteHeaderValue(this EndianBinaryWriter writer, int value, SizeId sizeId)
        {
            var sizeBytes = GetByteCount(sizeId);
            if (sizeBytes == 2)
            {
                sizeBytes = 4;
            }

            var valueBytes = GetBytes(value, sizeBytes);
            writer.Write((byte)sizeId);
            writer.Write(valueBytes);
        }

        private static byte[] GetBytes(int value, int size)
        {
            var converter = EndianBitConverter.Big;
            var bytes = converter.GetBytes(value);
            if (size == bytes.Length) return bytes;
            var offset = bytes.Length - size;

            var result = new byte[size];
            Array.Copy(bytes, offset, result, 0, size);
            return result;
        }

        public static int ReadHeaderValue(this EndianBinaryReader reader)
        {
            var sizeId = (SizeId) reader.ReadByte();
            var size = GetByteCount(sizeId);
            switch (size)
            {
                case 1:
                    return reader.ReadByte();
                case 2:
                case 4:
                    return reader.ReadInt32();
                default:
                    throw new Exception("Wrong data");
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
        Connect = 0x10,
        Type = 0x42,
        Time = 0x44,
        Body = 0x48,
        EBody = 0x49,
        Target = 0x46,
        Property = 0x70,
        Register = 0x71,
        Command = 0x72,
        PackSym = 0xB0,
        Length = 0xC3
    }

    public enum SizeId : byte
    {
        OneByte = 0xB1,
        TwoByte = 0xF0,
        FourByte = 0xF1
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

        protected override void WriteBody(EndianBinaryWriter writer)
        {
            writer.Write(Data, 0, Data.Length);
        }
    }

    public class GetPropertyPacket : RequestPacket
    {
        public string PropertyName { get; set; }

        public GetPropertyPacket(string propertyName) : base(Opcode.DataGetF)
        {
            PropertyName = propertyName;
        }

        protected override void WriteBody(EndianBinaryWriter writer)
        {
            writer.Write((byte)HeadId.Property);
            writer.WriteHeaderString(PropertyName);
        }
    }

    public class SetRegisterPacket : RequestPacket
    {
        public string RegisterName { get; }
        public int Value { get; }
        public SizeId SizeId { get; }

        public SetRegisterPacket(string registerName, int value, SizeId sizeId) : base(Opcode.DataGetF)
        {
            RegisterName = registerName;
            Value = value;
            SizeId = sizeId;
        }

        protected override void WriteBody(EndianBinaryWriter writer)
        {
            writer.Write((byte)HeadId.Register);
            writer.WriteHeaderString(RegisterName);
            writer.WriteHeaderValue(Value, SizeId);
        }
    }

    public class ConnectPacket : RequestPacket
    {
        private readonly ushort _bufferLength;

        public ConnectPacket(ushort bufferLength) : base(Opcode.Connect)
        {
            _bufferLength = bufferLength;
        }

        protected override void WriteBody(EndianBinaryWriter writer)
        {
            writer.Write((byte)HeadId.Connect);
            writer.Write((byte)0x00);
            writer.Write(_bufferLength);
        }
    }

    public class ChangeSpeedPacket : RequestPacket
    {
        public byte Divisor { get; set; }

        public ChangeSpeedPacket(byte divisor) : base(Opcode.ChangeSpeed)
        {
            Divisor = divisor;
        }

        protected override void WriteBody(EndianBinaryWriter writer)
        {
            writer.Write(Divisor);
        }
    }

    public abstract class SuccessPacket : ResponsePacket
    {
        public HeadId Type { get; }
        
        public SuccessPacket(ushort length, HeadId connect) : base(RespCode.Success)
        {
            Length = length;
            Type = connect;
        }
    }

    public class ConnectSuccessPacket : SuccessPacket
    {
        public ushort MaxPacketSize { get; set; }

        public ConnectSuccessPacket(ushort length) : base(length, HeadId.Connect)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
            var b = reader.ReadByte(); // Always 0x00
            Debug.Assert(b == 0);

            MaxPacketSize = reader.ReadUInt16();
        }
    }

    public class GenericSuccessPacket : SuccessPacket
    {
        public byte[] Data { get; set; }

        public GenericSuccessPacket(ushort length, HeadId connect) : base(length, connect)
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

    public class PropertySuccessPacket : SuccessPacket
    {
        public string PropertyName { get; set; }
        public string Data { get; set; }

        public PropertySuccessPacket(ushort length) : base(length, HeadId.Property)
        {
        }

        public override void Read(EndianBinaryReader reader)
        {
            PropertyName = reader.ReadHeaderString();
            var dataBytes = reader.ReadHeaderDataBytes();
            Data = Encoding.ASCII.GetString(dataBytes);
        }
    }

    public class SetRegisterSuccessPacket : SuccessPacket
    {
        public string RegisterName { get; set; }
        public int Value { get; set; }

        public SetRegisterSuccessPacket(ushort length) : base(length, HeadId.Register)
        {
            
        }

        public override void Read(EndianBinaryReader reader)
        {
            RegisterName = reader.ReadHeaderString();
            Value = reader.ReadHeaderValue();
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

        public RequestPacket(Opcode opcode)
        {
            Opcode = opcode;
        }
        
        protected abstract void WriteBody(EndianBinaryWriter writer);

        private byte[] GetBody()
        {
            var memoryStream = new MemoryStream();
            var writer = new EndianBinaryWriter(EndianBitConverter.Big, memoryStream);
            WriteBody(writer);
            writer.Flush();
            return memoryStream.ToArray();
        }

        public byte[] GetBytes()
        {
            var body = GetBody();

            var memoryStream = new MemoryStream();
            var writer = new EndianBinaryWriter(EndianBitConverter.Big, memoryStream);
            writer.Write((byte) Opcode);
            writer.Write((ushort) (body.Length + 3));
            writer.Write(body);
            writer.Flush();
            return memoryStream.ToArray();
        }
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
