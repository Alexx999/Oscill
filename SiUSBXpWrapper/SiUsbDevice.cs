using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Win32.SafeHandles;

namespace SiUSBXp
{
    public sealed class SiUsbDevice : Stream
    {
        private readonly SafeSiUsbHandle _device;
        private readonly Lazy<PartNumber> _partNumber;
        private readonly Lazy<string> _deviceProductString;
        private bool _disposed;

        public override bool CanRead { get; } = true;
        public override bool CanSeek { get; } = false;
        public override bool CanWrite { get; } = true;

        private SiUsbDevice(IntPtr handle)
        {
            _device = new SafeSiUsbHandle(handle);
            ThreadPool.BindHandle(_device);
            _partNumber = new Lazy<PartNumber>(GetPartNumber);
            _deviceProductString = new Lazy<string>(GetDeviceProductString);
        }


        public static uint NumDevices
        {
            get
            {
                uint result = 0;
                var code = SiUsbXpDll.SI_GetNumDevices(ref result);
                if (code == SiUsbXpDll.SI_DEVICE_NOT_FOUND)
                {
                    return 0;
                }
                ExceptionHelper.ThrowIfError(code);
                return result;
            }
        }

        public static Version DllVersion
        {
            get
            {
                uint high = 0;
                uint low = 0;
                var code = SiUsbXpDll.SI_GetDLLVersion(ref high, ref low);
                ExceptionHelper.ThrowIfError(code);
                return ParseVersion(high, low);
            }
        }

        public static Version DriverVersion
        {
            get
            {
                uint high = 0;
                uint low = 0;
                var code = SiUsbXpDll.SI_GetDriverVersion(ref high, ref low);
                ExceptionHelper.ThrowIfError(code);
                return ParseVersion(high, low);
            }
        }

        public static SiUsbDevice Open(uint device)
        {
            IntPtr handle = IntPtr.Zero;
            var code = SiUsbXpDll.SI_Open(device, ref handle);
            ExceptionHelper.ThrowIfError(code);
            return new SiUsbDevice(handle);
        }

        public static string GetProductString(uint device, ProductProperty options)
        {
            var sb = new StringBuilder(256);
            var code = SiUsbXpDll.SI_GetProductString(device, sb, (uint) options);
            ExceptionHelper.ThrowIfError(code);
            return sb.ToString();
        }

        public PartNumber PartNumber => _partNumber.Value;

        private PartNumber GetPartNumber()
        {
            ThrowIfDisposed();
            byte partNum = 0;
            var code = SiUsbXpDll.SI_GetPartNumber(NativeHandle, ref partNum);
            ExceptionHelper.ThrowIfError(code);
            return (PartNumber)partNum;
        }

        public string DeviceProductString => _deviceProductString.Value;

        private string GetDeviceProductString()
        {
            ThrowIfDisposed();
            var len = (byte)(SiUsbXpDll.SI_MAX_DEVICE_STRLEN - 1);
            var buffer = new byte[len * 2];
            var code = SiUsbXpDll.SI_GetDeviceProductString(NativeHandle, buffer, ref len, false);
            ExceptionHelper.ThrowIfError(code);
            return Encoding.Unicode.GetString(buffer, 0, (len + 1) * 2);
        }

        public IntPtr NativeHandle => _device.DangerousGetHandle();

        protected override void Dispose(bool disposing)
        {
            _disposed = true;
            if (disposing)
            {
                _device.Dispose();
            }
        }

        public void SetFlowControl(RxPinOption cts, TxPinOption rts, TxPinOption dtr, RxPinOption dcr, RxPinOption dcd, bool softwareFlowControl)
        {
            ThrowIfDisposed();
            var code = SiUsbXpDll.SI_SetFlowControl(NativeHandle, (byte) cts, (byte) rts, (byte) dtr, (byte) dcr,
                (byte) dcd, softwareFlowControl);
            ExceptionHelper.ThrowIfError(code);
        }

        public RxQueueState CheckRxQueue()
        {
            ThrowIfDisposed();
            uint numBytesInQueue = 0;
            uint numQueueStatus = 0;
            var code = SiUsbXpDll.SI_CheckRXQueue(NativeHandle, ref numBytesInQueue, ref numQueueStatus);
            ExceptionHelper.ThrowIfError(code);
            return new RxQueueState(numBytesInQueue, (RxQueueStatus) numQueueStatus);
        }

        public void SetBaudRate(uint rate)
        {
            ThrowIfDisposed();
            var code = SiUsbXpDll.SI_SetBaudRate(NativeHandle, rate);
            ExceptionHelper.ThrowIfError(code);
        }

        public void SetLineControl(StopBits stopBits, Parity parity, byte bitsPerWord)
        {
            ThrowIfDisposed();
            var stopVal = GetStopBitValue(stopBits);
            var parityVal = GetParityValue(parity);
            var result = bitsPerWord << 8;
            result |= parityVal << 4;
            result |= stopVal;
            var code = SiUsbXpDll.SI_SetLineControl(NativeHandle, (ushort) result);
            ExceptionHelper.ThrowIfError(code);
        }

        public byte ModemStatus
        {
            get
            {
                ThrowIfDisposed();
                byte result = 0;
                var code = SiUsbXpDll.SI_GetModemStatus(NativeHandle, ref result);
                ExceptionHelper.ThrowIfError(code);
                return result;
            }
        }

        public void SetBreak(bool breakState)
        {
            ThrowIfDisposed();
            var value = breakState ? 0 : 1;
            var code = SiUsbXpDll.SI_SetBreak(NativeHandle, (ushort) value);
            ExceptionHelper.ThrowIfError(code);
        }

        public byte ReadLatch()
        {
            ThrowIfDisposed();
            byte result = 0;
            var code = SiUsbXpDll.SI_ReadLatch(NativeHandle, ref result);
            ExceptionHelper.ThrowIfError(code);
            return result;
        }

        public void WriteLatch(byte mask, byte latch)
        {
            ThrowIfDisposed();
            var code = SiUsbXpDll.SI_WriteLatch(NativeHandle, mask, latch);
            ExceptionHelper.ThrowIfError(code);
        }

        private byte GetParityValue(Parity parity)
        {
            switch (parity)
            {
                case Parity.None:
                {
                    return 0;
                }
                case Parity.Odd:
                {
                    return 1;
                }
                case Parity.Even:
                {
                    return 2;
                }
                case Parity.Mark:
                {
                    return 3;
                }
                case Parity.Space:
                {
                    return 4;
                }
                default:
                {
                    throw new ArgumentException("Invalid Parity value", nameof(parity));
                }
            }
        }

        private byte GetStopBitValue(StopBits stopBits)
        {
            switch (stopBits)
            {
                case StopBits.One:
                {
                    return 0;
                }
                case StopBits.OnePointFive:
                {
                    return 1;
                }
                case StopBits.Two:
                {
                    return 2;
                }
                default:
                {
                    throw new ArgumentException("Invalid Stop Bits value", nameof(stopBits));
                }
            }
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
            {
                throw new ObjectDisposedException("SiUsbDevice");
            }
        }

        public static void SetTimeouts(uint read, uint write)
        {
            var code = SiUsbXpDll.SI_SetTimeouts(read, write);
            ExceptionHelper.ThrowIfError(code);
        }

        public static void SetTimeouts(Timeouts timeouts)
        {
            SetTimeouts(timeouts.Read, timeouts.Write);
        }

        public static Timeouts GetTimeouts()
        {
            var result = new Timeouts();
            var code = SiUsbXpDll.SI_GetTimeouts(ref result.Read, ref result.Write);
            ExceptionHelper.ThrowIfError(code);
            return result;
        }

        public override unsafe int Read(byte[] buffer, int offset, int count)
        {
            ThrowIfDisposed();
            uint result = 0;
            int code;
            fixed (byte* ptr = buffer)
            {
                code = SiUsbXpDll.SI_Read(NativeHandle, ptr + offset, (uint) count, ref result, IntPtr.Zero);
            }
            ExceptionHelper.ThrowIfError(code);
            return (int) result;
        }

        public override unsafe void Write(byte[] buffer, int offset, int count)
        {
            ThrowIfDisposed();
            uint result = 0;
            int code;
            fixed (byte* ptr = buffer)
            {
                code = SiUsbXpDll.SI_Write(NativeHandle, ptr + offset, (uint) count, ref result, IntPtr.Zero);
            }
            ExceptionHelper.ThrowIfError(code);
        }

        public override void Flush()
        {
            Flush(true, true);
        }

        public void Flush(bool flushTransmit, bool flushReceive)
        {
            ThrowIfDisposed();
            var code = SiUsbXpDll.SI_FlushBuffers(NativeHandle, flushTransmit, flushReceive);
            ExceptionHelper.ThrowIfError(code);
        }

        public override long Length
        {
            get { throw new NotSupportedException(); }
        }

        public override long Position
        {
            get { throw new NotSupportedException(); }
            set { throw new NotSupportedException(); }
        }

        public override void SetLength(long value)
        {
            throw new NotSupportedException();
        }

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotSupportedException();
        }

        private static Version ParseVersion(uint high, uint low)
        {
            var a = (high >> 16) & 0xFFFF;
            var b = high & 0xFFFF;
            var c = (low >> 16) & 0xFFFF;
            var d = low & 0xFFFF;
            return new Version((int) a, (int) b, (int) c, (int) d);
        }

        public override unsafe IAsyncResult BeginRead(byte[] buffer, int offset, int count, AsyncCallback callback, object state)
        {
            ThrowIfDisposed();
            SiUsbAsyncResult asyncResult = new SiUsbAsyncResult(0, buffer, _device, callback, state, false);
            NativeOverlapped* intOverlapped = asyncResult.OverLapped;
            uint result = 0;
            var code = SiUsbXpDll.SI_Read(NativeHandle, buffer, (uint) count, ref result, new IntPtr(intOverlapped));
            ExceptionHelper.ThrowIfError(code);

            return asyncResult;
        }

        public override int EndRead(IAsyncResult asyncResult)
        {
            SiUsbAsyncResult afsar = asyncResult as SiUsbAsyncResult;
            if (afsar == null || afsar.IsWrite)
            {
                throw new ArgumentException("Wrong AsyncResult", nameof(asyncResult));
            }

            if (1 == Interlocked.CompareExchange(ref afsar._EndXxxCalled, 1, 0))
            {
                throw new InvalidOperationException("EndRead called twice");
            }
            afsar.Wait();
            afsar.ReleaseNativeResource();

            if (afsar.ErrorCode != 0)
            {
                throw ExceptionHelper.CodeToException(afsar.ErrorCode);
            }
            return afsar.NumBytesRead;
        }

        public override unsafe IAsyncResult BeginWrite(byte[] buffer, int offset, int count, AsyncCallback callback, object state)
        {
            ThrowIfDisposed();
            SiUsbAsyncResult asyncResult = new SiUsbAsyncResult(0, buffer, _device, callback, state, true);
            NativeOverlapped* intOverlapped = asyncResult.OverLapped;
            uint result = 0;
            var code = SiUsbXpDll.SI_Write(NativeHandle, buffer, (uint)count, ref result, new IntPtr(intOverlapped));
            ExceptionHelper.ThrowIfError(code);

            return asyncResult;
        }

        public override void EndWrite(IAsyncResult asyncResult)
        {
            SiUsbAsyncResult afsar = asyncResult as SiUsbAsyncResult;
            if (afsar == null || !afsar.IsWrite)
            {
                throw new ArgumentException("Wrong AsyncResult", nameof(asyncResult));
            }

            if (1 == Interlocked.CompareExchange(ref afsar._EndXxxCalled, 1, 0))
            {
                throw new InvalidOperationException("EndWrite called twice");
            }
            afsar.Wait();
            afsar.ReleaseNativeResource();

            if (afsar.ErrorCode != 0)
            {
                throw ExceptionHelper.CodeToException(afsar.ErrorCode);
            }
        }
    }

    internal sealed unsafe class SiUsbAsyncResult : IAsyncResult
    {
        private AsyncCallback _userCallback;
        private object _userStateObject;
        private ManualResetEvent _waitHandle;
        private SafeSiUsbHandle _handle;


        private NativeOverlapped* _overlapped;
        internal NativeOverlapped* OverLapped => _overlapped;
        internal bool IsAsync => _overlapped != null;


        internal int _EndXxxCalled;
        private int _numBytes;
        internal int NumBytes => _numBytes;

        internal int ErrorCode { get; private set; }

        internal int NumBufferedBytes { get; set; }

        internal int NumBytesRead => _numBytes + NumBufferedBytes;

        private bool _isWrite;
        internal bool IsWrite => _isWrite;

        private bool _isComplete;     
        private bool _completedSynchronously;

        private static IOCompletionCallback s_IOCallback;

        
        internal SiUsbAsyncResult(
            int numBufferedBytes,
            byte[] bytes,
            SafeSiUsbHandle handle,
            AsyncCallback userCallback,
            object userStateObject,
            bool isWrite)
        {
            _userCallback = userCallback;
            _userStateObject = userStateObject;
            _isWrite = isWrite;
            NumBufferedBytes = numBufferedBytes;
            _handle = handle;

            ManualResetEvent waitHandle = new ManualResetEvent(false);
            _waitHandle = waitHandle;

            Overlapped overlapped = new Overlapped(0, 0, IntPtr.Zero, this);
            
            if (userCallback != null)
            {
                var ioCallback = s_IOCallback;
                if (ioCallback == null) s_IOCallback = ioCallback = AsyncFsCallback;
                _overlapped = overlapped.Pack(ioCallback, bytes);
            }
            else
            {
                _overlapped = overlapped.UnsafePack(null, bytes);
            }
        }

        internal static SiUsbAsyncResult CreateBufferedReadResult(int numBufferedBytes, AsyncCallback userCallback, object userStateObject, bool isWrite)
        {
            SiUsbAsyncResult asyncResult = new SiUsbAsyncResult(numBufferedBytes, userCallback, userStateObject, isWrite);
            asyncResult.CallUserCallback();
            return asyncResult;
        }

        private SiUsbAsyncResult(int numBufferedBytes, AsyncCallback userCallback, object userStateObject, bool isWrite)
        {
            _userCallback = userCallback;
            _userStateObject = userStateObject;
            _isWrite = isWrite;
            NumBufferedBytes = numBufferedBytes;
        }

        public bool IsCompleted => _isComplete;

        public WaitHandle AsyncWaitHandle
        {
            get
            {
                if (_waitHandle == null)
                {
                    ManualResetEvent mre = new ManualResetEvent(false);
                    if (_overlapped != null && _overlapped->EventHandle != IntPtr.Zero)
                    {
                        mre.SafeWaitHandle = new SafeWaitHandle(_overlapped->EventHandle, true);
                    }
                    
                    if (Interlocked.CompareExchange<ManualResetEvent>(ref _waitHandle, mre, null) == null)
                    {
                        if (_isComplete)
                            _waitHandle.Set();
                    }
                    else {
                        mre.Close();
                    }
                }
                return _waitHandle;
            }
        }
        public object AsyncState => _userStateObject;
        public bool CompletedSynchronously => _completedSynchronously;

        private void CallUserCallbackWorker()
        {
            _isComplete = true;

            Thread.MemoryBarrier();
            _waitHandle?.Set();

            _userCallback(this);
        }

        internal void CallUserCallback()
        {
            if (_userCallback != null)
            {
                _completedSynchronously = false;
                ThreadPool.QueueUserWorkItem(state => ((SiUsbAsyncResult)state).CallUserCallbackWorker(), this);
            }
            else {
                _isComplete = true;

                Thread.MemoryBarrier();
                _waitHandle?.Set();
            }
        }

        internal void ReleaseNativeResource()
        {
            if (_overlapped != null)
                Overlapped.Free(_overlapped);
        }

        internal void Wait()
        {
            if (_waitHandle == null) return;
            try
            {
                _waitHandle.WaitOne();
            }
            finally
            {
                _waitHandle.Close();
            }
        }

        private static void AsyncFsCallback(uint errorCode, uint numBytes, NativeOverlapped* pOverlapped)
        {
            Overlapped overlapped = Overlapped.Unpack(pOverlapped);

            SiUsbAsyncResult asyncResult =
                (SiUsbAsyncResult)overlapped.AsyncResult;
            asyncResult._numBytes = (int)numBytes;
            
            asyncResult.ErrorCode = (int)errorCode;

            asyncResult._completedSynchronously = false;
            asyncResult._isComplete = true;

            Thread.MemoryBarrier();

            ManualResetEvent wh = asyncResult._waitHandle;
            if (wh != null)
            {
                bool r = wh.Set();
                if (!r) throw new IOException();
            }

            AsyncCallback userCallback = asyncResult._userCallback;
            userCallback?.Invoke(asyncResult);
        }

        internal void Cancel()
        {
            if (IsCompleted)
                return;

            if (_handle.IsInvalid)
                return;
        }
    }

    public struct Timeouts
    {
        public uint Read;
        public uint Write;

        public Timeouts(uint read, uint write)
        {
            Read = read;
            Write = write;
        }
    }

    public enum ProductProperty
    {
        SerialNumber = SiUsbXpDll.SI_RETURN_SERIAL_NUMBER,
        Description = SiUsbXpDll.SI_RETURN_DESCRIPTION,
        LinkName = SiUsbXpDll.SI_RETURN_LINK_NAME,
        Vid = SiUsbXpDll.SI_RETURN_VID,
        Pid = SiUsbXpDll.SI_RETURN_PID
    }

    public struct RxQueueState
    {
        public RxQueueState(uint bytes, RxQueueStatus status)
        {
            BytesInQueue = bytes;
            QueueStatus = status;
        }

        public uint BytesInQueue { get; }
        public RxQueueStatus QueueStatus { get; }
    }

    public enum RxQueueStatus
    {
        Empty = SiUsbXpDll.SI_RX_EMPTY,
        Overrun = SiUsbXpDll.SI_RX_OVERRUN,
        Ready = SiUsbXpDll.SI_RX_READY
    }

    public enum RxPinOption : byte
    {
        StatusInput = SiUsbXpDll.SI_STATUS_INPUT,
        HandshakeLine = SiUsbXpDll.SI_HANDSHAKE_LINE
    }

    public enum TxPinOption : byte
    {
        HeldInactive = SiUsbXpDll.SI_HELD_INACTIVE,
        HeldActive = SiUsbXpDll.SI_HELD_ACTIVE,
        FirmwareControlled = SiUsbXpDll.SI_FIRMWARE_CONTROLLED,
        TransmitActiveSignal = SiUsbXpDll.SI_TRANSMIT_ACTIVE_SIGNAL
    }

    public enum PartNumber : byte
    {
        Invalid,
        CP2101 = SiUsbXpDll.SI_CP2101_VERSION,
        CP2102 = SiUsbXpDll.SI_CP2102_VERSION,
        CP2103 = SiUsbXpDll.SI_CP2103_VERSION,
        CP2104 = SiUsbXpDll.SI_CP2104_VERSION
    }
}
