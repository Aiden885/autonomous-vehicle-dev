#include "configure.h"

Configure::Configure()
{
    LoadBottomParam();
}
void Configure::LoadBottomParam() {
    std::string FilePath = "./file/insdrtk.ini";
    QSettings setting(QString::fromStdString(FilePath), QSettings::IniFormat);
    setting.beginReadArray("transform");
    INSDGPSParam.PortName = setting.value("PortName").toString();
    INSDGPSParam.PortNameRTK = setting.value("PortNameRTK").toString();
    INSDGPSParam.usr_pwd = setting.value("usr_pwd").toString();
    INSDGPSParam.NTripCasterIP = setting.value("NTripCasterIP").toString();
    INSDGPSParam.NTripCasterPort = setting.value("NTripCasterPort").toInt();
    setting.endArray();
}
