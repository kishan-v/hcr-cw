using UnityEngine;
using Microsoft.Win32;
using System;
using System.Collections;

public class OmniMovementCalibration : MonoBehaviour {

    public static float GetCalibrationValue()
    {
        float consumerCalibrationValue;

        RegistryKey calibrationKey = Registry.CurrentUser.OpenSubKey(@"Software\HeroVR\SDK\OmniYawOffset", false);
        if (calibrationKey == null)
        {
            Debug.LogError(System.DateTime.Now.ToLongTimeString() + ": OmniMovementCalibration(GetCalibrationValue) - You need to calibrate your Omni using the external calibration application");
            consumerCalibrationValue = 0.0f;
        }
        else
        {
            consumerCalibrationValue = Convert.ToSingle(calibrationKey.GetValue(""));
            calibrationKey.Close();
        }

        Debug.Log(System.DateTime.Now.ToLongTimeString() + ": OmniMovementCalibration(GetCalibrationValue) - Calibration Value = " + consumerCalibrationValue);
        return consumerCalibrationValue;
    }

    public static float GetCouplingPercentage()
    {
        float consumerCouplingPercentageValue;

        RegistryKey couplingPercentageKey = Registry.CurrentUser.OpenSubKey(@"Software\HeroVR\SDK\CouplingPercentage", false);
        if (couplingPercentageKey == null)
        {
            RegistryKey newKey;
            newKey = Registry.CurrentUser.CreateSubKey(@"Software\HeroVR\SDK\CouplingPercentage");
            newKey.SetValue("", 1.0f);
            consumerCouplingPercentageValue = 1.0f;
            newKey.Close();
        }
        else
        {
            consumerCouplingPercentageValue = Convert.ToSingle(couplingPercentageKey.GetValue(""));
            couplingPercentageKey.Close();
        }

        //Debug.Log(System.DateTime.Now.ToLongTimeString() + ": OmniMovementCalibration(GetCouplingPercentage) - Coupling Percentage = " + consumerCouplingPercentageValue);
        return consumerCouplingPercentageValue;

    }
}
