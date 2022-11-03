using System;
using Android.App;
using Android.Content;
using Android.Runtime;
using Android.Views;
using Android.Widget;
using Android.OS;
using axRobot.Core;
using System.Threading.Tasks;
using Android.Content.PM;
using Plugin.Permissions;
using Plugin.CurrentActivity;
using Android;
using AndroidX.Core.App;

namespace axRobot
{
    [Activity (Label = "axRobot", MainLauncher = true, Icon = "@drawable/ax", ScreenOrientation = ScreenOrientation.Portrait)]
	public class MainActivity : Activity
	{
		BluetoothLEHandler bh = new BluetoothLEHandler();
        Button forwardButton;
        Button backButton;
        Button leftButton;
        Button rightButton;
        Button stopButton;
        Button action1Button;
        Button action2Button;
        Button action3Button;
        Button action4Button;
        Button action6Button;
        Button plusButton;
        Button minusButton;
        Button oaButton;
        Button sensorRightButton;
        Button sensorLeftButton;
        Button sensorFrontButton;

        protected override async void OnCreate (Bundle bundle)
		{
			base.OnCreate (bundle);
            CrossCurrentActivity.Current.Init(this, bundle);

            // Set our view from the "main" layout resource
            SetContentView(Resource.Layout.Main);

            bindUi();

            GetPermissions();

            // attach events to the buttons
            forwardButton.Click += async delegate { await bh.ConnectAndSendChar('f'); };
            backButton.Click += async delegate { await bh.ConnectAndSendChar('b'); };
            leftButton.Click += async delegate { await bh.ConnectAndSendChar('l'); };
            rightButton.Click += async delegate { await bh.ConnectAndSendChar('r'); };
            stopButton.Click += async delegate { await bh.ConnectAndSendChar('s'); };
            plusButton.Click += async delegate { await bh.ConnectAndSendChar('+'); };
            minusButton.Click += async delegate { await bh.ConnectAndSendChar('-'); };
            oaButton.Click += async delegate { await bh.ConnectAndSendChar('o'); };
            sensorLeftButton.Click += async delegate { await bh.ConnectAndSendChar('y'); };
            sensorRightButton.Click += async delegate { await bh.ConnectAndSendChar('x'); };
            sensorFrontButton.Click += async delegate { await bh.ConnectAndSendChar('c'); };

            //Define what happens when clicking on button "1"
            action1Button.Click += async delegate { await bh.ConnectAndSendChar('1'); };
            action2Button.Click += async delegate { await bh.ConnectAndSendChar('2'); };
            action3Button.Click += async delegate { await bh.ConnectAndSendChar('3'); };
            action4Button.Click += async delegate { await bh.ConnectAndSendChar('4'); };
            action6Button.Click += async delegate { await bh.ConnectAndSendChar('6'); };

            //TODO: Define what happens when clicking on button "4"..."5" in axRobo.ino
    	}

        #region helper methods

        protected override async void OnStart()
        {
            base.OnStart();
            await bh.StartScanningForDevices();
            await bh.Connect();
        }

        protected void GetPermissions()
        {
            ActivityCompat.RequestPermissions(this, new String[] { Manifest.Permission.AccessCoarseLocation, Manifest.Permission.AccessFineLocation, Manifest.Permission.Bluetooth, Manifest.Permission.BluetoothAdmin, Manifest.Permission.BluetoothScan, Manifest.Permission.BluetoothConnect }, 0);
        }

        public override void OnRequestPermissionsResult(int requestCode, string[] permissions, [GeneratedEnum] Android.Content.PM.Permission[] grantResults)
        {
            PermissionsImplementation.Current.OnRequestPermissionsResult(requestCode, permissions, grantResults);
            base.OnRequestPermissionsResult(requestCode, permissions, grantResults);
        }

        protected void bindUi()
        {
            // Get our buttons from the layout resource
            forwardButton = FindViewById<Button> (Resource.Id.forwardButton);
            backButton = FindViewById<Button> (Resource.Id.backButton);
            leftButton = FindViewById<Button> (Resource.Id.leftButton);
            rightButton = FindViewById<Button> (Resource.Id.rightButton);
            stopButton = FindViewById<Button> (Resource.Id.stopButton);
            plusButton = FindViewById<Button>(Resource.Id.plusButton);
            minusButton = FindViewById<Button>(Resource.Id.minusButton);
            oaButton = FindViewById<Button>(Resource.Id.oaButton);
            sensorLeftButton = FindViewById<Button>(Resource.Id.sensorLeftButton);
            sensorRightButton = FindViewById<Button>(Resource.Id.sensorRightButton);
            sensorFrontButton = FindViewById<Button>(Resource.Id.sensorFrontButton);
            action1Button = FindViewById<Button> (Resource.Id.action1Button);
            action2Button = FindViewById<Button> (Resource.Id.action2Button);
            action3Button = FindViewById<Button> (Resource.Id.action3Button);
            action4Button = FindViewById<Button> (Resource.Id.action4Button);
            action6Button = FindViewById<Button> (Resource.Id.action6Button);
        }
        #endregion
	}
}


