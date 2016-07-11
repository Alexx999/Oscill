using System;
using Microsoft.Win32.SafeHandles;

namespace SiUSBXp
{
    internal sealed class SafeSiUsbHandle : SafeHandleMinusOneIsInvalid
    {

        public SafeSiUsbHandle(IntPtr preexistingHandle) : base(true)
        {
            SetHandle(preexistingHandle);
        }

        public SafeSiUsbHandle(IntPtr preexistingHandle, bool ownsHandle) : base(ownsHandle)
        {
            SetHandle(preexistingHandle);
        }

        protected override bool ReleaseHandle()
        {
            return SiUsbXpDll.SI_Close(handle) == SiUsbXpDll.SI_SUCCESS;
        }
    }
}
