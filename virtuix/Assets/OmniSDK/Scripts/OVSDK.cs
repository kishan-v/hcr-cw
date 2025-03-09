using UnityEngine;
using System.Runtime.InteropServices;
using System.IO;
using System;
using UnityEngine.UI;
using UnityEngine.XR;
using System.Xml;
using System.Collections.Generic;

public class OVSDK : MonoBehaviour
{
	public static string _SDKVersion = "0.4.0";

	public delegate void OVSDKEventCallback(int nMsgType, string sMsgContent);
	public delegate void OVSDKBuyCallback(string sItem, string sOutTradeNo, string sInTradeNo, string sErr);
	public delegate void OVSDKSaveGameDataCallback(int nError, string sMsg);
	public delegate void OVSDKLoadGameDataCallback(int nError, string sMsg, IntPtr data, int len);

	static OVSDKEventCallback _DllEventCallback = null;

	[StructLayout(LayoutKind.Sequential)]
	public struct DeviceInfo
	{
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		public string sNo;      //Omni serial-number;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string sUID;     //Omni device UID in Omniverse backend.
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string sShopName; //Shop name
		[MarshalAs(UnmanagedType.U4)]
		public int nId;         //for internal usage
		[MarshalAs(UnmanagedType.U4)]
		public int nShop;       //Shop Id
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string sCategory;//for internal usage
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct UserInfo
	{
		[MarshalAs(UnmanagedType.U4)]
		public int nUserId;             //The player's unique account id in Omniverse;
		[MarshalAs(UnmanagedType.U4)]
		public int nGameId;             //GameID from OVSDK::Init 

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
		string sCookies;                //for internal usage 

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 4)]
		public string sGameSDKType;     //Omniverse SDK type, "unty": unity-SDK, "ue4x": UE4-SDK;
		[MarshalAs(UnmanagedType.U4)]
		public int nGameSDKVersion;     //Omniverse SDK version, version string 0xAAAA.0xBB.0xCC, format as uint32 0xAAAABBCC;

		[MarshalAs(UnmanagedType.U4)]
		public int nGamePrepareLeft;    //Game-prepare time left, use for single game ticket mode;
		[MarshalAs(UnmanagedType.U4)]
		public int nGameDurationLeft;   //Game-play time left;
		[MarshalAs(UnmanagedType.U4)]
		public int nGamePrepare;        //Game-prepare time, each game can config its own prepare time. The time is used to choose level, match game, but the duration is limited.
										//If level or match start, you should call OVSDK.ConfirmPlayGame() to tell SDK countdown game-time now.
		[MarshalAs(UnmanagedType.U4)]
		public int nGameDuration;       //Gameplay time;

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
		public string sBillingMode;     //Ticket mode, "timing", "timingreal", "direct_game", "shiyu_coin", "game_auth", "timescount";
		[MarshalAs(UnmanagedType.U4)]
		public int nUserProp;           //for internal usage;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		public string sConsolePath;     //for internal usage;

		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public char[] nCoupleRate;      //Omni couple rate data of default
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public char[] nUserCoupleRate;  //Omni couple rate setting about user

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 60)]
		string sReserved;               //for internal usage;

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string sQrcode;          //The omniverse trade number for this game ticket;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		public string sWeb2d;           //for internal usage;

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string sReserved2;       //for internal usage;

		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		public string sUserName;            //Player's nick game in Omniverse;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
		public string sUserPhone;           //Player's phone number registered in Omniverse; (maybe masked, some character replaced by '*' for privacy)
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		public string sUserEmail;           //Player's email registered in Omniverse; (maybe masked)
		public double nUserBalance;          //Player's balance in Omniverse; (In-game purchase costs this balance)
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 124)]
		public string sUserIcon;            //Player's portrait icon url;
		[MarshalAs(UnmanagedType.U4)]
		public int nDesktopDurationLeft;//for internal usage; 
	}

	// public interface for Game-developer
	static bool _bIniting = false, _bInitDone = false, _bJustInitDone = false;
	static string _sInitResult;
	static int _nInitResultCode = 0;
	static OVSDK _Instance = null;
	public static OVSDK instance
	{
		get {
			if (null == _Instance)
			{
				_Instance = GameObject.FindObjectOfType<OVSDK>();
				if (null == _Instance)
				{
					GameObject go = new GameObject("OVSDK_GlobalInstance");
					_Instance = go.AddComponent<OVSDK>();
					DontDestroyOnLoad(go);
				}
			}

			return _Instance;
		}
	}
    public static void SetCallBackOnMsgFromSDK(OVSDKEventCallback cbHvMsg)
    {
        _DllEventCallback = cbHvMsg;
    }
	public static void Init(int nGameId, string sGameKey, string sParam)
	{
		if (null == OVSDK.instance) {
			Debug.LogError("get OVSDK.instance failed.");
		}

		//if (_bIniting || _bInitDone) {
		//	Debug.LogError("OVSDK should Init once only.");
		//}

		_bIniting = true;
		_bInitDone = _bJustInitDone = false;

		sParam = sParam + ";sdk_type=unty;sdk_version=" + _SDKVersion;
		DllInit(nGameId, sGameKey, sParam, onEventFromSDK, IntPtr.Zero);
	}

	public static bool HasInitialized()
	{
		return _bInitDone;
	}

	public static bool JustInitialized()
	{
		return _bJustInitDone;
	}

	public static int GetInitResultCode()
	{
		return _nInitResultCode;
	}

	public static string GetInitResult()
	{
		return _sInitResult;
	}

    public static DeviceInfo GetDeviceInfo()
    {
        DeviceInfo info = (DeviceInfo)Marshal.PtrToStructure(DllGetDeviceInfo(), typeof(DeviceInfo)); ;
        return info;
    }

    public static UserInfo GetUserInfo()
    {
        UserInfo info = (UserInfo)Marshal.PtrToStructure(DllGetUserInfo(), typeof(UserInfo));
        return info;
    }

	public static bool IsGuest()
	{
		return DllIsGuest();
	}

    private static void onEventFromSDK(IntPtr sType, int nRetCode, IntPtr sRet, int nLen, IntPtr pUserData)
    {
		//string type = bytesToString(sType);
		string type = Marshal.PtrToStringAnsi(sType);
		string ret = "";

		//ret = bytesToString(sRet, nLen);
		ret = Marshal.PtrToStringAnsi(sRet, nLen);
		Debug.Log("OVSDK.OnEventFromSDK(" + type + ", " + nRetCode + ",\n" + ret + ")");

		if (type == "init")
		{
			_bInitDone = true;
			_sInitResult = ret;
			_nInitResultCode = nRetCode;			
		}
    }
    

	public static float GetOmniYawOffset()
	{
		return DllGetOmniYawOffset();
	}

	public static float GetOmniCoupleRate()
	{
		return DllGetOmniCoupleRate();
	}

	public static UInt32 GetUserOmniCoupleRate()
	{
		return DllGetUserOmniCoupleRate();
	}

	public static void SetOmniCoupleRate(float rate)
	{
		DllSetOmniCoupleRate(rate);
	}
	public static void SetOmniCoupleMode(bool useCoupleMode)
	{
		DllSetOmniCoupleMode(useCoupleMode);
	}

	//----------------------------------------------------------------------------------------------------------------------------------
	// Omniverse.Functions.dll functions.
	delegate void DllCallback(IntPtr sType, int nRetCode, IntPtr sRet, int nLen, IntPtr pUserData);
	[DllImport("Omniverse.Functions")]
	static extern bool DllInit(int nGameId, string sGameKey, string sParam, DllCallback cb, IntPtr pUserData);

	[DllImport("Omniverse.Functions")]
	static extern void DllShutdown();

	[DllImport("Omniverse.Functions")]
	public static extern bool DllLaunchGame(string exe, string workdir, string cmdline, int game_id, int prepare_time);   //启动游戏，传入参数

	[DllImport("Omniverse.Functions")]
	static extern bool DllIsDevMode();

	[DllImport("Omniverse.Functions")]
	static extern void DllDrive();

	[DllImport("Omniverse.Functions")]
	static extern void DllSendCommand(int nCmd, string sData, int nLen);

	[DllImport("Omniverse.Functions")]
	static extern void DllBuy(string sItem, double nPrice, string sOutTradeNo);

	[DllImport("Omniverse.Functions")]
	static extern IntPtr DllGetDeviceInfo();

	[DllImport("Omniverse.Functions")]
	static extern IntPtr DllGetUserInfo();

	[DllImport("Omniverse.Functions")]
	static extern float DllGetOmniYawOffset();

	[DllImport("Omniverse.Functions")]
	static extern float DllGetOmniCoupleRate();

	[DllImport("Omniverse.Functions")]
	static extern UInt32 DllGetUserOmniCoupleRate();

	[DllImport("Omniverse.Functions")]
	static extern void DllSetOmniCoupleRate(float coupleRate);

	[DllImport("Omniverse.Functions")]
	static extern void DllSetOmniCoupleMode(bool useCoupleMode);

	[DllImport("Omniverse.Functions")]
	static extern bool DllIsGuest();

	[DllImport("Omniverse.Functions")]
	static extern void DllSaveGameData(IntPtr data, int len);

	[DllImport("Omniverse.Functions")]
	static extern void DllLoadGameData();

	[DllImport("Omniverse.Functions")]
	public static extern void DllTest(IntPtr param);

#if UNITY_EDITOR
	[DllImport("User32.dll", SetLastError = true, ThrowOnUnmappableChar = true, CharSet = CharSet.Auto)]
	public static extern int MessageBox(IntPtr handle, String message, String title, int type);
#endif

	//----------------------------------------------------------------------------------------------------------------------------------
	// internal utility functions.
	
	void Start()
	{
	}
	void Update()
	{
		DllDrive();

		if (_bInitDone)
		{
			if (_bIniting)
			{
				_bIniting = false;
				_bJustInitDone = true;
			}
			else if (_bJustInitDone)
			{
				_bJustInitDone = false;
			}
		}
	}
	void OnDestroy()
	{
		Debug.Log("OVSDK shutdown.");
		DllShutdown();
	}
}