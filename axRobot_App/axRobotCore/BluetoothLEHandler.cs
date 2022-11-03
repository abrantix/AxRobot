using System;
using System.Linq;
using System.Threading.Tasks;
using Plugin.BLE;
using Plugin.BLE.Abstractions.Contracts;
using System.Threading;
using System.Reflection;
using System.Drawing;

namespace axRobot.Core
{
	public class BluetoothLEHandler
    {
        public IDevice SelectedDevice { get; private set; }
        public ICharacteristic SelectedCharacteristic { get; private set; }
        public static readonly byte[] ServiceId = new byte[] { 0xE0, 0xFF };
        public static readonly byte[] CharacteristicId = new byte[] { 0xE1, 0xFF };
        public bool IsConnected { get; private set; }
        public bool IsScanning { get; private set; }
        private string LastSeenDeviceAddress;

        SemaphoreSlim connectSemaphoreSlim = new SemaphoreSlim(1);
        SemaphoreSlim ioSemaphoreSlim = new SemaphoreSlim(1);
        SemaphoreSlim scanningSemaphoreSlim = new SemaphoreSlim(1);
        public BluetoothLEHandler()
		{
            IsConnected = false;
            IsScanning = false;
                CrossBluetoothLE.Current.Adapter.DeviceDiscovered += Adapter_DeviceDiscovered;
                CrossBluetoothLE.Current.Adapter.ScanTimeoutElapsed += Adapter_ScanTimeoutElapsed;
                CrossBluetoothLE.Current.Adapter.DeviceConnectionLost += Adapter_DeviceConnectionLost;
                CrossBluetoothLE.Current.Adapter.DeviceDisconnected += Adapter_DeviceDisconnected;
                CrossBluetoothLE.Current.Adapter.ScanTimeout = 10000;
        }

        private void Adapter_DeviceDisconnected(object sender, Plugin.BLE.Abstractions.EventArgs.DeviceEventArgs e)
        {
            if (SelectedDevice != null)
            {
                if (e.Device.Id == SelectedDevice.Id)
                {
                    SelectedDevice = null;
                    Disconnect().Wait();
                }
            }
        }

        private void Adapter_DeviceConnectionLost(object sender, Plugin.BLE.Abstractions.EventArgs.DeviceErrorEventArgs e)
        {
            if (SelectedDevice != null)
            {
                if (e.Device.Id == SelectedDevice.Id)
                {
                    SelectedDevice = null;
                    Disconnect().Wait();
                }
            }
        }

        public async Task StartScanningForDevices()
        {
            try
            {
                await scanningSemaphoreSlim.WaitAsync();
                if (!IsScanning)
                {
                    IsScanning = true;
                    await CrossBluetoothLE.Current.Adapter.StartScanningForDevicesAsync();
                }
            }
            finally
            {
                scanningSemaphoreSlim.Release();
            }
        }

        private void Adapter_ScanTimeoutElapsed(object sender, EventArgs e)
        {
            IsScanning = false;
            if (CrossBluetoothLE.Current.Adapter.IsScanning)
            {
                CrossBluetoothLE.Current.Adapter.StopScanningForDevicesAsync().Wait();
            }
            if (SelectedDevice == null)
            {
                StartScanningForDevices().Wait();
            }
        }

        private static bool IsBleDeviceOfInterest(string name)
        {
            return name.Contains("BT05")
            || name.Contains("AT-09")
            || name.Contains("JDY-09")
            || name.Contains("HMSoft");
        }

        private static string GetNativeDeviceAddress(IDevice device)
        {
            PropertyInfo propInfo = device.NativeDevice.GetType().GetProperty("Address");
            string address = (string)propInfo.GetValue(device.NativeDevice, null);
            return address;
        }

        private void Adapter_DeviceDiscovered(object sender, Plugin.BLE.Abstractions.EventArgs.DeviceEventArgs e)
        {
            if (e.Device.Name != null && SelectedDevice == null && IsBleDeviceOfInterest(e.Device.Name) 
                    //always reconnect to the same BLE device if there was any connection before
                && (string.IsNullOrEmpty(LastSeenDeviceAddress) || GetNativeDeviceAddress(e.Device) == LastSeenDeviceAddress)
               )
            {
                SelectedDevice = e.Device;
                LastSeenDeviceAddress = GetNativeDeviceAddress(SelectedDevice);
                CrossBluetoothLE.Current.Adapter.StopScanningForDevicesAsync().Wait();
                IsScanning = false;
            }
        }

        public async Task<bool> Connect()
        {
            try
            {
                await ioSemaphoreSlim.WaitAsync();
                if (!IsConnected && SelectedDevice != null)
                {
                    CancellationTokenSource tokenSource = new CancellationTokenSource(5000);
                    await CrossBluetoothLE.Current.Adapter.ConnectToDeviceAsync(SelectedDevice, new Plugin.BLE.Abstractions.ConnectParameters(), tokenSource.Token);
                    var services = await SelectedDevice.GetServicesAsync();
                    if (services != null)
                    {
                        var service = services.FirstOrDefault(x => x.Id.ToByteArray().Take(2).SequenceEqual(ServiceId));
                        if (service != null)
                        {
                            var characteristics = await service.GetCharacteristicsAsync();
                            if (characteristics != null)
                            {
                                var characteristic = characteristics.FirstOrDefault(x => x.Id.ToByteArray().Take(2).SequenceEqual(CharacteristicId));
                                if (characteristic != null)
                                {
                                    SelectedCharacteristic = characteristic;
                                    IsConnected = true;
                                    return true;
                                }
                            }
                        }
                    }
                }
                else
                {
                    await StartScanningForDevices();
                }
            }
            finally
            {
                ioSemaphoreSlim.Release();
            }

            return false;
        }

        public async Task Disconnect()
        {
            if (SelectedDevice != null)
            {
                if (!CrossBluetoothLE.Current.Adapter.IsScanning)
                {
                    await CrossBluetoothLE.Current.Adapter.DisconnectDeviceAsync(SelectedDevice);
                }
            }
            SelectedDevice = null;
            SelectedCharacteristic = null;
            IsConnected = false;
            IsScanning = false;
        }

        public async Task<bool> ConnectAndSendChar(char c)
        {
            bool gotSemaphore = false;
            try
            {
                gotSemaphore = await connectSemaphoreSlim.WaitAsync(5000);
                if (gotSemaphore)
                {
                    if (IsConnected)
                    {
                        return await SendChar(c);
                    }
                    else
                    {
                        await Connect();
                        if (IsConnected)
                        {
                            return await SendChar(c);
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
                else
                {
                    return false;
                }
            }
            finally
            {
                if (gotSemaphore)
                {
                    connectSemaphoreSlim.Release();
                }
            }
        }

        private async Task<bool> SendChar(char c)
        {
            bool success = false;
            if (SelectedCharacteristic != null)
            {
                try
                {
                    CancellationTokenSource tokenSource = new CancellationTokenSource(2000);
                    success = await SelectedCharacteristic.WriteAsync(new byte[] { (byte)c }, tokenSource.Token);
                }
                catch
                {
                    success = false;
                    await Disconnect();
                }
            }
            return success;
        }
    }
}

