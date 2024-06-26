﻿using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete
{
    internal class GeozillaCoreDll
    {
#if DEBUG
        private const string ConfigName = "Debug";
#else
        private const string ConfigName = "Release";
#endif

        private const string DllName = $"../../../../../geozilla-core/bin/{ConfigName}/geozilla-core.dll";

        [DllImport(DllName, EntryPoint = "?GenerateGeoJsonBuffer@@YAPEBDPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GenerateGeoJsonBuffer([MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DllName, EntryPoint = "?FreeBuffer@@YAXPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern void FreeBuffer(IntPtr buffer);
    }
}
