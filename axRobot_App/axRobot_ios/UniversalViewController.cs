using axRobot.Core;
using Foundation;
using System;
using System.Threading.Tasks;
using UIKit;

namespace axRobot_ios
{
    public partial class UniversalViewController : UIViewController
    {
        BluetoothLEHandler bh = new BluetoothLEHandler();
        public UniversalViewController() //: base("UniversalViewController", null)
        {
            
        }

        /*public UniversalViewController(IntPtr handle) : base(handle)
        {

        }*/

        public override void ViewDidLoad()
        {
            base.ViewDidLoad();
            AttachButtons();
        }

        public void AttachButtons()
        {
            ForwardButton.TouchDown += async delegate { await bh.ConnectAndSendChar('f'); };
            BackButton.TouchDown += async delegate { await bh.ConnectAndSendChar('b'); };
            LeftButton.TouchDown += async delegate { await bh.ConnectAndSendChar('l'); };
            RightButton.TouchDown += async delegate { await bh.ConnectAndSendChar('r'); };
            StopButton.TouchDown += async delegate { await bh.ConnectAndSendChar('s'); };
            AccelerateButton.TouchDown += async delegate { await bh.ConnectAndSendChar('+'); };
            DecelerateButton.TouchDown += async delegate { await bh.ConnectAndSendChar('-'); };
            ObstacleAvoidanceButton.TouchDown += async delegate { await bh.ConnectAndSendChar('o'); };

            //Define what happens when clicking on button "1"..."6"
            Action1Button.TouchDown += async delegate { await bh.ConnectAndSendChar('1'); };
            Action2Button.TouchDown += async delegate { await bh.ConnectAndSendChar('2'); };
            Action3Button.TouchDown += async delegate { await bh.ConnectAndSendChar('3'); };
            Action4Button.TouchDown += async delegate { await bh.ConnectAndSendChar('4'); };
            Action5Button.TouchDown += async delegate { await bh.ConnectAndSendChar('6'); };
            //Action6Button.TouchDown += async delegate { await bh.ConnectAndSendChar('6'); };

            L.TouchDown += async delegate { await bh.ConnectAndSendChar ('y'); };
            F.TouchDown += async delegate { await bh.ConnectAndSendChar ('x'); };
            R.TouchDown += async delegate { await bh.ConnectAndSendChar ('c'); };
        }

        partial void UIButtonjgSwXKZ8_TouchUpInside (UIButton sender)
        {
            return;
        }
    }
}