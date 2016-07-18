using System;

namespace SiUSBXp
{
    internal static class ExceptionHelper
    {
        public static Exception CodeToException(int code)
        {
            switch (code)
            {
                case SiUsbXpDll.SI_SUCCESS:
                {
                    throw new Exception("CodeToException called on success status code");
                }
                case SiUsbXpDll.SI_DEVICE_NOT_FOUND:
                {
                    return new DeviceNotFoundException();
                }
                case SiUsbXpDll.SI_INVALID_HANDLE:
                {
                    return new InvalidHandleException();
                }
                case SiUsbXpDll.SI_READ_ERROR:
                {
                    return new ReadErrorException();
                }
                case SiUsbXpDll.SI_RX_QUEUE_NOT_READY:
                {
                    return new RxQueueNotReadyException();
                }
                case SiUsbXpDll.SI_WRITE_ERROR:
                {
                    return new WriteErrorException();
                }
                case SiUsbXpDll.SI_RESET_ERROR:
                {
                    return new ResetErrorException();
                }
                case SiUsbXpDll.SI_INVALID_PARAMETER:
                {
                    return new InvalidParameterException();
                }
                case SiUsbXpDll.SI_INVALID_REQUEST_LENGTH:
                {
                    return new InvalidRequestLengthException();
                }
                case SiUsbXpDll.SI_DEVICE_IO_FAILED:
                {
                    return new DeviceIoFailedException();
                }
                case SiUsbXpDll.SI_INVALID_BAUDRATE:
                {
                    return new InvalidBaudrateException();
                }
                case SiUsbXpDll.SI_FUNCTION_NOT_SUPPORTED:
                {
                    return new FunctionNotSupportedException();
                }
                case SiUsbXpDll.SI_GLOBAL_DATA_ERROR:
                {
                    return new GlobalDataErrorException();
                }
                case SiUsbXpDll.SI_SYSTEM_ERROR_CODE:
                {
                    return new SystemErrorCodeException();
                }
                case SiUsbXpDll.SI_READ_TIMED_OUT:
                {
                    return new ReadTimedOutException();
                }
                case SiUsbXpDll.SI_WRITE_TIMED_OUT:
                {
                    return new WriteTimedOutException();
                }
                case SiUsbXpDll.SI_IO_PENDING:
                {
                    throw new Exception("CodeToException called on success status code");
                }
                default:
                {
                    return new SiUsbException(code);
                }
            }
        }

        public static void ThrowIfError(int code)
        {
            if(!IsError(code)) return;

            throw CodeToException(code);
        }

        public static bool IsError(int code)
        {
            return !(code == SiUsbXpDll.SI_SUCCESS || code == SiUsbXpDll.SI_IO_PENDING);
        }
    }

    public class WriteTimedOutException : SiUsbException
    {
        public WriteTimedOutException() : base(SiUsbXpDll.SI_WRITE_TIMED_OUT, "Write Timed Out")
        {
        }
    }

    public class ReadTimedOutException : SiUsbException
    {
        public ReadTimedOutException() : base(SiUsbXpDll.SI_READ_TIMED_OUT, "Read Timed Out")
        {
        }
    }

    public class SystemErrorCodeException : SiUsbException
    {
        public SystemErrorCodeException() : base(SiUsbXpDll.SI_SYSTEM_ERROR_CODE, "System Error Code")
        {
        }
    }

    public class GlobalDataErrorException : SiUsbException
    {
        public GlobalDataErrorException() : base(SiUsbXpDll.SI_GLOBAL_DATA_ERROR, "Global Data Error")
        {
        }
    }

    public class FunctionNotSupportedException : SiUsbException
    {
        public FunctionNotSupportedException() : base(SiUsbXpDll.SI_FUNCTION_NOT_SUPPORTED, "Function Not Supported")
        {
        }
    }

    public class InvalidBaudrateException : SiUsbException
    {
        public InvalidBaudrateException() : base(SiUsbXpDll.SI_INVALID_BAUDRATE, "Invalid Baudrate")
        {
        }
    }

    public class DeviceIoFailedException : SiUsbException
    {
        public DeviceIoFailedException() : base(SiUsbXpDll.SI_DEVICE_IO_FAILED, "Device IO Failed")
        {
        }
    }

    public class InvalidRequestLengthException : SiUsbException
    {
        public InvalidRequestLengthException() : base(SiUsbXpDll.SI_INVALID_REQUEST_LENGTH, "Invalid Request Length")
        {
        }
    }

    public class InvalidParameterException : SiUsbException
    {
        public InvalidParameterException() : base(SiUsbXpDll.SI_INVALID_PARAMETER, "Invalid Parameter")
        {
        }
    }

    public class ResetErrorException : SiUsbException
    {
        public ResetErrorException() : base(SiUsbXpDll.SI_RESET_ERROR, "Reset Error")
        {
        }
    }

    public class WriteErrorException : SiUsbException
    {
        public WriteErrorException() : base(SiUsbXpDll.SI_WRITE_ERROR, "Write Error")
        {
        }
    }

    public class RxQueueNotReadyException : SiUsbException
    {
        public RxQueueNotReadyException() : base(SiUsbXpDll.SI_RX_QUEUE_NOT_READY, "Rx Queue Not Ready")
        {
        }
    }

    public class ReadErrorException : SiUsbException
    {
        public ReadErrorException() : base(SiUsbXpDll.SI_READ_ERROR, "Read Error")
        {
        }
    }

    public class InvalidHandleException : SiUsbException
    {
        public InvalidHandleException() : base(SiUsbXpDll.SI_INVALID_HANDLE, "Invalid Handle")
        {
        }
    }

    public class DeviceNotFoundException : SiUsbException
    {
        public DeviceNotFoundException() : base(SiUsbXpDll.SI_DEVICE_NOT_FOUND, "Device Not Found")
        {
        }
    }

    public class SiUsbException : Exception
    {
        public SiUsbException(int code) : base($"Error Code {code:X}")
        {
            HResult = code;
        }

        public SiUsbException(int code, string message) : base(message)
        {
            HResult = code;
        }
    }
}
