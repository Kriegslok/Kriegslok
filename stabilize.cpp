
#include <QDebug>
#include <QDataStream>
#include <QtMath>
#include <QObject>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdint.h>
#include <QFile>
#include <QLineF>
#include <QTextStream>
#include <QSettings>
#include <QCoreApplication>

#include "mavlink.h"
#include "structs.h"
#include "calcattpid.h"
#include "stabilize.h"
#include "readcfg.h"

using namespace domain;

Stabilize::Stabilize(QObject *parent) : QObject(parent)
{
    cfgName = "/Cfg.cfg";

//    RollPID.ElevTkoffPitch = 0;
//    RollPID.KElevTurn = 0;
//    YawPID.ElevTkoffPitch = 0;
    YTR_PID.ElevTkoffPitch = 0;
    ZPU_PID.ElevTkoffPitch = 0;
    ATP_PID.Kv = 0;
    ATT_PID.Kv = 0;
    VTP_PID.Kv = 0;
    VTT_PID.Kv = 0;

    RollError.Error = 0;
    RollError.ErrorPrev = 0;
    RollError.ErrorPrePrev = 0;
    RollError.IntegralPrev = 0;
    RollError.Integral = 0;
    RollError.Differential = 0;

    PitchError.Error = 0;
    PitchError.ErrorPrev = 0;
    PitchError.ErrorPrePrev = 0;
    PitchError.IntegralPrev = 0;
    PitchError.Integral = 0;
    PitchError.Differential = 0;

    YawError.Error = 0;
    YawError.ErrorPrev = 0;
    YawError.ErrorPrePrev = 0;
    YawError.IntegralPrev = 0;
    YawError.Integral = 0;
    YawError.Differential = 0;

    YTR_Error.Error = 0;
    YTR_Error.ErrorPrev = 0;
    YTR_Error.ErrorPrePrev = 0;
    YTR_Error.IntegralPrev = 0;
    YTR_Error.Integral = 0;
    YTR_Error.Differential = 0;

    ZPU_Error.Error = 0;
    ZPU_Error.ErrorPrev = 0;
    ZPU_Error.ErrorPrePrev = 0;
    ZPU_Error.IntegralPrev = 0;
    ZPU_Error.Integral = 0;
    ZPU_Error.Differential = 0;

    ATP_Error.Error = 0;
    ATP_Error.ErrorPrev = 0;
    ATP_Error.ErrorPrePrev = 0;
    ATP_Error.IntegralPrev = 0;
    ATP_Error.Integral = 0;
    ATP_Error.Differential = 0;

    ATT_Error.Error = 0;
    ATT_Error.ErrorPrev = 0;
    ATT_Error.ErrorPrePrev = 0;
    ATT_Error.IntegralPrev = 0;
    ATT_Error.Integral = 0;
    ATT_Error.Differential = 0;

    VTP_Error.Error = 0;
    VTP_Error.ErrorPrev = 0;
    VTP_Error.ErrorPrePrev = 0;
    VTP_Error.IntegralPrev = 0;
    VTP_Error.Integral = 0;
    VTP_Error.Differential = 0;


    VTT_Error.Error = 0;
    VTT_Error.ErrorPrev = 0;
    VTT_Error.ErrorPrePrev = 0;
    VTT_Error.IntegralPrev = 0;
    VTT_Error.Integral = 0;
    VTT_Error.Differential = 0;

   // PitchAttPidCalcResult.AttPidCalcResult = 0;


//    ERROR ATP_Error; //Составляющая тангажа от высоты
//    ERROR ATT_Error; //Составляющая газа от высоты
//    ERROR VTP_Error; //Составляющая тангажа от скорости
//    ERROR VTT_Error; //Составляющая газа от высоты

    //Test parameters
    testV = 19; // Воздушная скорость
    testAlt = 110; //Высота
    //Переменные параметры алгоритма

    TakeOffIsChecked = 0;
    prev_time_boot_ms = 2; // Предыдущее значение времени
    pre_prev_time_boot_ms = 1;
    relativeTime = 0; // мс, Время с момента запуска программы
    deployParachuteTime = 0;
    deployParachuteCount = 0;

    GS = 0; //Путевая скорость

    parameters.throttleArm = true;
    parameters.useUBX = true;
    parameters.useNMEA = true;
    parameters.modeTest = false;
    parameters.takeOff = true;
    parameters.doPlaneLand = false;
    parameters.doParachuteLand = false;


    PWM = 0; //ШИМ сигнал на выходе ПИДа

    PropPart = 0;
    IntPart = 0;
    DifPart = 0;
    KompensPart = 0;

    AileronPWM = 0; //Сигнал отклонения рулевой машинки элеронов
    ElevatorPWM = 0;
    RudderPWM = 0;
    ThrottlePWM = 0;
    ParachutePWM = -1;


    DesRoll = 0; //Заданный крен
    DesPitch = 0; //Заданный тангаж
    DesYaw = 0; // Заданный курс, не используется

    DesAlt = 0; //Заданная высота
    DesTrackAngle = 0; //ЗПУ
    DesTrackAnglePrev = 0;




    //Навигационные переменные

    pixGPLat = 56.58; //Инерциальная широта
    pixGPLon = 38.27; //Инерциальная долгота
    pixGPhdg = 0; //Инерциальный курс
    pixGPLatm = 0; //Инерциальная широта m
    pixGPLonm = 0; //Инерциальная долгота
    pixGPLatPrev = 0;
    pixGPLonPrev = 0;
    pixGPLatPrevm = 0;
    pixGPLonPrevm = 0;
    pixGPPsi = 0;

    pixSNSLat = 0; //СНС широта
    pixSNSLon = 0; //СНС долгота
    pixSNSVel = 0; //СНС путевая скорость
    pixSNSFix = 3; //СНС fix
    pixSNSNumSat = 6; //СНС Кол-во спутников
    pixSNSLatm = 0; //СНС широта
    pixSNSLonm = 0; //СНС долгота
    pixSNSLatPrev = 0;
    pixSNSLonPrev = 0;
    pixSNSLatPrevm = 0;
    pixSNSLonPrevm = 0;
    pixSNSPsi = 0;
    deltapixSNSPsi = 0;
    SNSPermit = 1;

    NmeaLat = 0;
    NmeaLon = 0;
    NmeaLatm = 0;
    NmeaLonm = 0;
    NmeaLatPrevm = 0;
    NmeaLonPrevm = 0;
    NmeaHdop = 0;
    NmeaNsat = 0;
    deltaNmeaLatm = 0;
    deltaNmeaLonm = 0;
    deltaNmeaPsi_active = 0;
    NmeaPsi = 0; //Вычисленный путевой угол СНС
    deltaNmeaPsi = 0;
    NmeaPsiMean = 0;

    photoTime = 0; //time_boot_ms на момент съемки
    outerPermit = 0;

    navLat = 0; //Поправленное значение широты
    navLon = 0; //Поправленное значение доглоты
    navHdg = 0; //Поправленное значение курса
    navVel = 0; //Поправленное значение скорости
    navTrackAngle = 0; //Поправленное значение путевого угла
    navLatm = 0; //Поправленное значение широты
    navLonm = 0; //Поправленное значение доглоты
    navLatmPrev = 0; //Предыдущее значение поправленной широты
    navLonmPrev = 0;
    navtimeMsReal = 3; //Начальное значение текущего отсчета времени, используемого в навигации
    navtimeMsPrev = 2; //Начальное значение предыдущего отсчета времени, используемого в навигации
    navtimeMsPrePrev = 1;

     Lppm = 500; //Расстояние по прямой до текущего ППМ
     R = 500; //Расстояние до проекции на линию пути (траверза)
     Letap = 500; //Длина этапа

    arspd = 0; //Воздушная скорость
    hudGSpeed = 0; //Путевая скорость из интерфейса
    hudBaroAlt = 0; //Баровысота
    hudHdg = 0; //Магнитный курс

    deltapixSNSLatm = 0;
    deltapixSNSLonm = 0;
    deltapixSNSPsi_active = 0;
    deltaouterLatm = 0;
    deltaouterLonm = 0;
    deltaouterPsi_active = 0;
    deltaBm = 0;
    deltaLm = 0;

    //readFromUAVCfg();
   // ReadCfg::readUAVCfg();
    ReadCfg(); //Читаем параметры из файла


    ReadWaypoints(); // Считываем полетное задание из файла


    DesV = pidV.Vkr; //Крейсерская скорость
    WP.actualWP = 2;
    WP.prevWP = 1;
    WP.cosPsiTs = 0;

//    double B = 56.904800;
//    double L = 37.682700;
//    poTransformWgs84ToSample->Transform(1, &L, &B);

//    //____________________________________________
//    //FOR TEST
//    qDebug() << QString("metr SK-42: %1 %2").arg(L, 0,'f',2).arg(B, 0,'f',2);

//    B = 6280962.20;
//    L = 7459146.13;
//    poTransformSampleToWgs84->Transform(1, &L, &B);
//    qDebug() << QString("degree WGS-84: %1 %2").arg(L, 0,'f',8).arg(B, 0,'f',8);
//    //_____________________________________________


    // Создание и запись в файл параметров телеметрии
    file.setFileName(QCoreApplication::applicationDirPath() + "/AP2_log.txt");
    file.open(QFile::WriteOnly);
    if(file.isOpen())
    {
        QTextStream stream(&file);
        stream
                << "Lon m GP inert." <<  "\t"
                << "Lat m GP inert." << "\t"
                << "Lon m NMEA corrector" << "\t"
                << "Lat m NMEA corrector" << "\t"
                << "Lon m AP2 counted" << "\t"
                << "Lat m AP2 counted" << "\t"
                << "Psi GP" << "\t"
                << "Psi GP Mean" << "\t"
                << "Psi NMEA" << "\t"
                << "Psi NMEA Mean" << "\t"
                << "WP" << "\t"
                << "NavTrackAngle" << "\t"
                << "DesTrackAngle" << "\t"
                << "GS" << "\t"
                << "Roll"<< "\t"
                << "DesRoll" << "\t"
                << "AileronPWM" << "\t"
                << "Pitch" << "\t"
                << "DesPitch" << "\t"
                << "ElevatorPWM" << "\t"
                << "Yaw" << "\t"
                << "DesTrackAngle" << "\t"
                << "RudderPWM" << "\t"
                << "Airspeed" << "\t"
                << "DesAirspeed" << "\t"
                << "ThrottlePWM" << "\t"
                << "Alt" << "\t"
                << "DesAlt" << "\n";


    }

} // Stabilize::Stabilize(QObject *parent)




    Stabilize::~Stabilize()
    {

    }

    void Stabilize::ReadCfg() {
        QSettings settings(QCoreApplication::applicationDirPath() +  cfgName, QSettings::IniFormat);
        createCoordinateTransformation(settings.value("prjFileName").toString());

         pidV.Vint = settings.value("Stab_parameters_common/Vint", 12).toFloat();

         RollPID.Kp = settings.value("Stab_parameters_aileronPID1/Kp", 1.2).toFloat();
         RollPID.Ki = settings.value("Stab_parameters_aileronPID1/Ki", 0.04).toFloat();
         RollPID.Kd = settings.value("Stab_parameters_aileronPID1/Kd", 0).toFloat();

         RollPID.KOmega = settings.value("Stab_parameters_aileronPID2/Ksp", 0).toFloat();
         RollPID.LimErrorMin = settings.value("Stab_parameters_aileronPID2/LimErrorMin", -1).toFloat();
         RollPID.LimErrorMax = settings.value("Stab_parameters_aileronPID2/LimErrorMax", 1).toFloat();
         RollPID.LimPWMMin = settings.value("Stab_parameters_aileronPID2/LimPWMMin", -1).toFloat();
         RollPID.LimPWMMax = settings.value("Stab_parameters_aileronPID2/LimPWMMax", 1).toFloat();
         RollPID.LimIntMin = settings.value("Stab_parameters_aileronPID2/LimIntMin", -0.4).toFloat();
         RollPID.LimIntMax = settings.value("Stab_parameters_aileronPID2/LimIntMax", 0.4).toFloat();

         PitchPID.Kp = settings.value("Stab_parameters_elevatorPID1/Kp", 1.2).toFloat();
         PitchPID.Ki = settings.value("Stab_parameters_elevatorPID1/Ki", 0.04).toFloat();
         PitchPID.Kd = settings.value("Stab_parameters_elevatorPID1/Kd", 0).toFloat();
         PitchPID.KOmega = settings.value("Stab_parameters_elevatorPID1/Kpd", 0).toFloat();

         PitchPID.KElevTurn = settings.value("Stab_parameters_elevatorPID2/KTurn", 0).toFloat();
         PitchPID.LimErrorMin = settings.value("Stab_parameters_elevatorPID2/LimErrorMin", -1).toFloat();
         PitchPID.LimErrorMax = settings.value("Stab_parameters_elevatorPID2/LimErrorMax", 1).toFloat();
         PitchPID.LimPWMMin = settings.value("Stab_parameters_elevatorPID2/LimPWMMin", -1).toFloat();
         PitchPID.LimPWMMax = settings.value("Stab_parameters_elevatorPID2/LimPWMMax", 1).toFloat();
         PitchPID.LimIntMin = settings.value("Stab_parameters_elevatorPID2/LimIntMin", -0.4).toFloat();
         PitchPID.LimIntMax = settings.value("Stab_parameters_elevatorPID2/LimIntMax", 0.4).toFloat();

         YawPID.Kp = settings.value("Stab_parameters_rudderPID1/RudderKp", 1.2).toFloat();
         YawPID.Ki = settings.value("Stab_parameters_rudderPID1/RudderKi", 0.00).toFloat();
         YawPID.Kd = settings.value("Stab_parameters_rudderPID1/RudderKd", 0).toFloat();

         YawPID.KOmega = settings.value("Stab_parameters_rudderPID2/Kpd", 0.001).toFloat();
         YawPID.LimErrorMin = settings.value("Stab_parameters_rudderPID2/LimErrorMin", -1).toFloat();
         YawPID.LimErrorMax = settings.value("Stab_parameters_rudderPID2/LimErrorMax", 1).toFloat();
         YawPID.LimPWMMin = settings.value("Stab_parameters_rudderPID2/LimPWMMin", -1).toFloat();
         YawPID.LimPWMMax = settings.value("Stab_parameters_rudderPID2/LimPWMMax", 1).toFloat();


         YTR_PID.Kp = settings.value("Nav_parameters_YTR1/Kp", 0.25).toFloat();
         YTR_PID.Ki = settings.value("Nav_parameters_YTR1/Ki", 0.001).toFloat();
         YTR_PID.Kd = settings.value("Nav_parameters_YTR1/Kd", 0.0001).toFloat();

         YTR_PID.LimErrorMin = settings.value("Nav_parameters_YTR2/LimErrorMin", -3).toFloat();
         YTR_PID.LimErrorMax = settings.value("Nav_parameters_YTR2/LimErrorMax", 3).toFloat();
         YTR_PID.LimPWMMin = settings.value("Nav_parameters_YTR2/DesRollMin", -0.52).toFloat();
         YTR_PID.LimPWMMax = settings.value("Nav_parameters_YTR2/DesRollMax", 0.52).toFloat();
         YTR_PID.LimIntMin = settings.value("Nav_parameters_YTR2/LimIntMin", -0.1).toFloat();
         YTR_PID.LimIntMax = settings.value("Nav_parameters_YTR2/LimIntMax", 0.1).toFloat();



         ZPU_PID.Kp = settings.value("Nav_parameters_ZPU1/Kp", 0.25).toFloat();
         ZPU_PID.Ki = settings.value("Nav_parameters_ZPU1/Ki", 0.001).toFloat();
         ZPU_PID.Kd = settings.value("Nav_parameters_ZPU1/Kd", 0.0001).toFloat();

         ZPU_PID.LimErrorMin = settings.value("Nav_parameters_ZPU2/LBUMin", -50).toFloat();
         ZPU_PID.LimErrorMax = settings.value("Nav_parameters_ZPU2/LBUMax", 50).toFloat();
         ZPU_PID.LimPWMMin = settings.value("Nav_parameters_ZPU2/ZPUMin", -0.52).toFloat();
         ZPU_PID.LimPWMMax = settings.value("Nav_parameters_ZPU2/ZPUMax", 0.52).toFloat();
         ZPU_PID.LimIntMin = settings.value("Nav_parameters_ZPU2/LimIntMin", -0.1).toFloat();
         ZPU_PID.LimIntMax = settings.value("Nav_parameters_ZPU2/LimIntMax", 0.1).toFloat();

         Navkoeff.RLur = settings.value("Nav_parameters_LUR/RLur", 80).toFloat();
         Navkoeff.RLurMin = settings.value("Nav_parameters_LUR/RLurMin", 100).toFloat();
         Navkoeff.RLurMax = settings.value("Nav_parameters_LUR/RLurMax", 300).toFloat();

         desPitchLimitMin = settings.value("Nav_parameters_DesPitch/Min", -1).toFloat();
         desPitchLimitMax = settings.value("Nav_parameters_DesPitch/Max", 1).toFloat();

         ATP_PID.Kp = settings.value("Nav_parameters_ATP1/Kp", 0.04).toFloat();
         ATP_PID.Ki = settings.value("Nav_parameters_ATP1/Ki", 0.001).toFloat();
         ATP_PID.Kd = settings.value("Nav_parameters_ATP1/Kd", 0.00001).toFloat();
         ATP_PID.Kv = settings.value("Nav_parameters_ATP1/Kv", -0.5).toFloat();


         ATP_PID.LimErrorMin = settings.value("Nav_parameters_ATP2/LimErrorMin", -20).toFloat();
         ATP_PID.LimErrorMax = settings.value("Nav_parameters_ATP2/LimErrorMax", 20).toFloat();
         ATP_PID.LimPWMMin = settings.value("Nav_parameters_ATP2/OutputMin", -0.8).toFloat();
         ATP_PID.LimPWMMax = settings.value("Nav_parameters_ATP2/OutputMax", 0.8).toFloat();
         ATP_PID.LimIntMin = settings.value("Nav_parameters_ATP2/LimIntMin", -1).toFloat();
         ATP_PID.LimIntMax = settings.value("Nav_parameters_ATP2/LimIntMax", 1).toFloat();

         ATT_PID.Kp = settings.value("Nav_parameters_ATT1/Kp", 0.4).toFloat();
         ATT_PID.Ki = settings.value("Nav_parameters_ATT1/Ki", 0.04).toFloat();
         ATT_PID.Kd = settings.value("Nav_parameters_ATT1/Kd", 0.00001).toFloat();
         ATT_PID.Kv = settings.value("Nav_parameters_ATT1/Kv", -0.5).toFloat();


         ATT_PID.LimErrorMin = settings.value("Nav_parameters_ATT2/LimErrorMin", -10).toFloat();
         ATT_PID.LimErrorMax = settings.value("Nav_parameters_ATT2/LimErrorMax", 10).toFloat();
         ATT_PID.LimPWMMin = settings.value("Nav_parameters_ATT2/OutputMin", -1).toFloat();
         ATT_PID.LimPWMMax = settings.value("Nav_parameters_ATT2/OutputMax", 1).toFloat();
         ATT_PID.LimIntMin = settings.value("Nav_parameters_ATT2/LimIntMin", -10).toFloat();
         ATT_PID.LimIntMax = settings.value("Nav_parameters_ATT2/LimIntMax", 10).toFloat();

         VTP_PID.Kp = settings.value("Nav_parameters_VTP1/Kp", 0.06).toFloat();
         VTP_PID.Ki = settings.value("Nav_parameters_VTP1/Ki", 0.001).toFloat();
         VTP_PID.Kd = settings.value("Nav_parameters_VTP1/Kd", 0.00001).toFloat();
         VTP_PID.Kv = settings.value("Nav_parameters_VTP1/Kv", 0.5).toFloat();

         VTP_PID.LimErrorMin = settings.value("Nav_parameters_VTP2/LimErrorMin", -10).toFloat();
         VTP_PID.LimErrorMax = settings.value("Nav_parameters_VTP2/LimErrorMax", 10).toFloat();
         VTP_PID.LimPWMMin = settings.value("Nav_parameters_VTP2/OutputMin", -1).toFloat();
         VTP_PID.LimPWMMax = settings.value("Nav_parameters_VTP2/OutputMax", 1).toFloat();
         VTP_PID.LimIntMin = settings.value("Nav_parameters_VTP2/LimIntMin", -1).toFloat();
         VTP_PID.LimIntMax = settings.value("Nav_parameters_VTP2/LimIntMax", 1).toFloat();

         VTT_PID.Kp = settings.value("Nav_parameters_VTT1/Kp", 0.4).toFloat();
         VTT_PID.Ki = settings.value("Nav_parameters_VTT1/Ki", 0.04).toFloat();
         VTT_PID.Kd = settings.value("Nav_parameters_VTT1/Kd", 0.00001).toFloat();
         VTT_PID.Kv = settings.value("Nav_parameters_VTT1/Kv", -0.5).toFloat();

         VTT_PID.LimErrorMin = settings.value("Nav_parameters_VTT2/LimErrorMin", -10).toFloat();
         VTT_PID.LimErrorMax = settings.value("Nav_parameters_VTT2/LimErrorMax", 10).toFloat();
         VTT_PID.LimPWMMin = settings.value("Nav_parameters_VTT2/OutputMin", -1).toFloat();
         VTT_PID.LimPWMMax = settings.value("Nav_parameters_VTT2/OutputMax", 1).toFloat();
         VTT_PID.LimIntMin = settings.value("Nav_parameters_VTT2/LimIntMin", -10).toFloat();
         VTT_PID.LimIntMax = settings.value("Nav_parameters_VTT2/LimIntMax", 10).toFloat();

         PitchPID.ElevTkoffTrim = settings.value("TkOff_parameters/ElevTkoffTrim", 0).toFloat();
         PitchPID.ElevTkoffPitch = settings.value("TkOff_parameters/ElevTkoffPitch", 0).toFloat();
         TkOffAltCheck = settings.value("TkOff_parameters/TkOffAltCheck", 30).toFloat();

         landThrottleCutoffAlt = settings.value("Plane_land_parameters/ThrottleCutoffAlt", 10).toFloat();
         landFlarePitch = settings.value("Plane_land_parameters/FlarePitch", 7).toFloat();
         landFlareAlt = settings.value("Plane_land_parameters/FlareAlt", 3).toFloat();

        pidV.Vmin = settings.value("LA_Spd_parameters/Vmin", 10).toFloat();
        pidV.Vkr = settings.value("LA_Spd_parameters/Vkr", 21).toFloat();
        pidV.Vmax = settings.value("LA_Spd_parameters/Vmax", 35).toFloat();
        pidV.Vland = settings.value("LA_Spd_parameters/Vland", 14).toFloat();

        deployParachuteDelay = settings.value("Parachute_parameters/DeployParachuteDelay", 3000).toInt();
        deployParachuteDuration = settings.value("Parachute_parameters/DeployParachuteDuration", 3000).toInt();
        deployParachuteReopen = settings.value("Parachute_parameters/DeployParachuteReopen", 3).toInt();
        deployParachuteReopenSPD = settings.value("Parachute_parameters/DeployParachuteReopenSPD", 10).toFloat();
        deployParachuteEnable = settings.value("Parachute_parameters/DeployParachuteReopenEnable", 10).toBool();
        releaseParachuteVSPD = settings.value("Parachute_parameters/ReleaseParachuteVSPD", 10).toFloat();

        fsBatLevel = settings.value("Failsafe_parameters_Bat/FsBatLevel", 100).toFloat();
        fsBatAction = settings.value("Failsafe_parameters_Bat/FsBatAction", 0).toInt();

        fsVVel = settings.value("Failsafe_parameters_VVel/FsVVel", 10).toFloat();
        fsVVelDeployParachute = settings.value("Failsafe_parameters_VVel/FsVVelDeployParachute", 10).toBool();

        fsSnsNoFixTime = settings.value("Failsafe_parameters_Sns/FsSnsNoFixTime", 10).toFloat();
        fsSnsOperatorDisable = settings.value("Failsafe_parameters_Sns/FsSnsOperatorDisable", 10).toBool();
        fsSnsAction = settings.value("Failsafe_parameters_Sns/FsSnsAction", 0).toInt();

        fsArspdNotActiveTime = settings.value("Failsafe_parameters_Airspeed/FsAirspeedNotActiveTime", 10).toFloat();
        fsArspdFixThrottleCheck = settings.value("Failsafe_parameters_Airspeed/FsAirspeedFixThrottleCheck", 10).toBool();
        fsArspdFixThrottle = settings.value("Failsafe_parameters_Airspeed/FsAirspeedFixThrottle", 0).toInt();
        fsArspdFixPitchCheck = settings.value("Failsafe_parameters_Airspeed/FsAirspeedFixPitchCheck", 10).toBool();
        fsArspdFixPitch = settings.value("Failsafe_parameters_Airspeed/FsAirspeedFixPitch", 10).toFloat();
        fsArspdAction = settings.value("Failsafe_parameters_Airspeed/FsAirspeedAction", 0).toInt();

        aileronPWMLimits.minpwm = settings.value("pwmLimits_aileron/minpwm", 1000).toInt();
        aileronPWMLimits.maxpwm = settings.value("pwmLimits_aileron/maxpwm", 2000).toInt();
        aileronPWMLimits.trim = settings.value("pwmLimits_aileron/trim", 1500).toInt();
        aileronPWMLimits.inverse = settings.value("pwmLimits_aileron/inverse", 1500).toBool();

        elevatorPWMLimits.minpwm = settings.value("pwmLimits_elevator/minpwm", 1000).toInt();
        elevatorPWMLimits.maxpwm = settings.value("pwmLimits_elevator/maxpwm", 2000).toInt();
        elevatorPWMLimits.trim = settings.value("pwmLimits_elevator/trim", 1500).toInt();
        elevatorPWMLimits.inverse = settings.value("pwmLimits_elevator/inverse", 1500).toBool();

        rudderPWMLimits.minpwm = settings.value("pwmLimits_rudder/minpwm", 1000).toInt();
        rudderPWMLimits.maxpwm = settings.value("pwmLimits_rudder/maxpwm", 2000).toInt();
        rudderPWMLimits.trim = settings.value("pwmLimits_rudder/trim", 1500).toInt();
        rudderPWMLimits.inverse = settings.value("pwmLimits_rudder/inverse", 1500).toBool();

        throttlePWMLimits.minpwm = settings.value("pwmLimits_throttle/minpwm", 1000).toInt();
        throttlePWMLimits.maxpwm = settings.value("pwmLimits_throttle/maxpwm", 2000).toInt();
        throttlePWMLimits.trim = settings.value("pwmLimits_throttle/trim", 1500).toInt();
        throttlePWMLimits.inverse = settings.value("pwmLimits_throttle/inverse", 1500).toBool();

        parachutePWMLimits.minpwm = settings.value("pwmLimits_parachute/minpwm", 1000).toInt();
        parachutePWMLimits.maxpwm = settings.value("pwmLimits_parachute/maxpwm", 2000).toInt();
        parachutePWMLimits.trim = settings.value("pwmLimits_parachute/trim", 1500).toInt();
        parachutePWMLimits.inverse = settings.value("pwmLimits_parachute/inverse", 1500).toBool();


        ATT_PID.KOmega = settings.value("Nav_Parameters/ATTKpd", 0).toFloat(); //Не используется
        ATT_PID.KElevTurn = settings.value("Nav_Parameters/ATTKTurn", 0).toFloat(); //Не используется
        ATT_PID.ElevTkoffTrim = settings.value("Nav_Parameters/ATTTkoffTrim", 0).toFloat(); //Не используется
        ATT_PID.ElevTkoffPitch = settings.value("TkOff_parameters/ElevTkoffPitch", 0).toFloat();//Не используется
        VTP_PID.ElevTkoffTrim = settings.value("Nav_Parameters/VTPTkoffTrim", 0).toFloat(); //Не используется
        VTP_PID.ElevTkoffPitch = settings.value("TkOff_parameters/ElevTkoffPitch", 0).toFloat(); //Не используется
        VTP_PID.KOmega = settings.value("Nav_Parameters/VTPKpd", 0).toFloat(); //Не используется
        VTP_PID.KElevTurn = settings.value("Nav_Parameters/VTPKTurn", 0).toFloat(); //Не используется
        VTT_PID.ElevTkoffTrim = settings.value("Nav_Parameters/VTTTkoffTrim", 0).toFloat(); //Не используется
        VTT_PID.ElevTkoffPitch = settings.value("TkOff_parameters/ElevTkoffPitch", 0).toFloat();//Не используется
        VTT_PID.KOmega = settings.value("Nav_Parameters/VTTKpd", 0).toFloat(); //Не используется
        VTT_PID.KElevTurn = settings.value("Nav_Parameters/VTTKTurn", 0).toFloat(); //Не используется


        //Navkoeff.Rnzcht = settings.value("Nav_Parameters/Rnzcht", 1000).toFloat();
        Navkoeff.windspd = settings.value("Meteo_Data/windSpeed", 0).toFloat();
        Navkoeff.windcourse = settings.value("Meteo_Data/windDirection", 0).toFloat();
        Navkoeff.airTemperature = settings.value("Meteo_Data/airTemperature", 0).toFloat();





        //WP.actualWP = settings.value("WP/AWP", 10).toInt();
        //         ATP_PID.KOmega = settings.value("Nav_Parameters/ATPKpd", 0).toFloat(); //Не используется
        //          ATP_PID.KElevTurn = settings.value("Nav_Parameters/ATPKTurn", 0).toFloat(); //Не используется
        //         ATP_PID.ElevTkoffTrim = settings.value("Nav_Parameters/ATPTkoffTrim", 0).toFloat(); //Не используется
        //         ATP_PID.ElevTkoffPitch = settings.value("TkOff_parameters/ElevTkoffPitch", 0).toFloat();//Не используется
        ATP_PID.KOmega = 0;
        ATP_PID.KElevTurn = 0;
        ATP_PID.ElevTkoffTrim = 0;
        ATP_PID.ElevTkoffPitch = 0;
        //         ZPU_PID.ElevTkoffTrim = settings.value("Nav_Parameters/ZPUTkoffTrim", 0).toFloat(); //Не используется
        //         ZPU_PID.KOmega = settings.value("Nav_Parameters/ZPUKOmega", 0).toFloat(); //Не используется
        //         ZPU_PID.KElevTurn = settings.value("Nav_Parameters/ZPUKTurn", 0).toFloat(); //Не используется
        ZPU_PID.ElevTkoffTrim = 0;
        ZPU_PID.KOmega = 0;
        ZPU_PID.KElevTurn = 0;
        //         YTR_PID.KOmega = settings.value("Nav_Parameters/YawToRollKpd", 0).toFloat(); //Не используется
        //         YTR_PID.KElevTurn = settings.value("Nav_Parameters/YawToRollKTurn", 0).toFloat(); //Не используется
        //         YTR_PID.ElevTkoffTrim = settings.value("Nav_Parameters/YawToRollTkoffTrim", 0).toFloat(); //Не используется
        YTR_PID.KOmega = 0;
        YTR_PID.KElevTurn = 0;
        YTR_PID.ElevTkoffTrim = 0;
        //         YawPID.KElevTurn = settings.value("Stab_parameters/RudderKTurn", 0).toFloat(); //Не используется
        //         YawPID.LimIntMin = settings.value("Stab_parameters/RudderLimin", 0).toFloat(); //Не используется
        //         YawPID.LimIntMax = settings.value("Stab_parameters/RudderLimiv", 0).toFloat(); //Не используется
        //         YawPID.ElevTkoffTrim = settings.value("TkOff_parameters/RudderTkoffTrim", 0).toFloat(); //Не используется
        YawPID.KElevTurn = 0;
        YawPID.LimIntMin = 0;
        YawPID.LimIntMax = 0;
        YawPID.ElevTkoffTrim = 0;

//       //WP.actualWP = settings.value("WP/AWP", 10).toInt();
    }

// ФУНКЦИЯ СТАБИЛИЗАЦИИ ПОЛЕТА. Срабатывает каждый раз, когда приходит пакет от датчика углов Pixhawk.
    void Stabilize::AttitudeSlot(float time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {

    if (TakeOffIsChecked == 1) //Если зачтён взлёт, то предвзлётное положение руля высоты равно 0, если нет, то оно ненулевое (заданное).
    {
        PitchPID.ElevTkoffTrim = 0;
    }
    else
    {
      QSettings settings(QCoreApplication::applicationDirPath() +  "/Cfg.cfg", QSettings::IniFormat);
      PitchPID.ElevTkoffTrim = settings.value("TkOff_parameters/ElevTkoffTrim", 0).toFloat();
    }

    this->currentPitch = pitch;
    this->currentRoll = roll;
    this->currentYaw = yaw;
    this->currentDesRoll = DesRoll;
    this->currentTimeBootMs = time_boot_ms;
    this->currentAltitude = hudBaroAlt;
    this->currentAirspeed = arspd;
    this->currentDesV = DesV;
    this->currentDesAlt = DesAlt;

    //Вызов функции вычисления ШИМ элеронов
    CalcAttPid::calculateAttPID(RollAttPidCalcResult, RollPID, pidV, RollError, arspd, relativeTime, time_boot_ms, prev_time_boot_ms, pre_prev_time_boot_ms, roll, DesRoll, yawspeed);
    AileronPWM = RollAttPidCalcResult.AttPidCalcResult;
    //Вызов функции вычисления ШИМ руля высоты
    DesPitch = qBound(desPitchLimitMin, DesPitch, desPitchLimitMax);
    this->currentDesPitch = DesPitch;
    CalcAttPid::calculateAttPID(PitchAttPidCalcResult, PitchPID, pidV, PitchError, arspd, relativeTime, time_boot_ms, prev_time_boot_ms, pre_prev_time_boot_ms, pitch, roll, DesPitch, pitchspeed);
    ElevatorPWM = PitchAttPidCalcResult.AttPidCalcResult;
    //Вызов функции вычисления ШИМ руля направления
    CalcAttPid::calculateAttPID(YawAttPidCalcResult, YawPID, pidV, YawError, arspd, relativeTime, time_boot_ms, prev_time_boot_ms, pre_prev_time_boot_ms, yaw, DesTrackAngle, DesTrackAngle);
    RudderPWM = YawAttPidCalcResult.AttPidCalcResult;
    qDebug() << "Des Pitch ATT Slot" << DesPitch << "Des Alt ATT Slot" << DesAlt << "Des V ATT Slot" << DesV;

    AileronPWMOut = calcPWM(aileronPWMLimits, AileronPWM);
    ElevatorPWMOut = calcPWM(elevatorPWMLimits, ElevatorPWM);
    RudderPWMOut = calcPWM(rudderPWMLimits, RudderPWM);
    ThrottlePWMOut = calcPWM(throttlePWMLimits, ThrottlePWM);
    ParachutePWMOut = calcPWM(parachutePWMLimits, ParachutePWM);
    ThrottlePWMoutFl = qBound (0.0, (ThrottlePWMOut/1000.0), 1.0);
    this->currentThrottle = ThrottlePWMoutFl * 100;
    qDebug()<<"Aileron" << AileronPWMOut <<"Elevator"<< ElevatorPWMOut <<"Throttle" << ThrottlePWMOut << "ThrottleOut" << ThrottlePWMoutFl;


    if (prev_time_boot_ms > 2) //Счёт относительного времени для задержки вычисления значения интегралов и дифференциалов при включении автопилота
    {
     relativeTime = relativeTime + (time_boot_ms - prev_time_boot_ms);
    }
    pre_prev_time_boot_ms = prev_time_boot_ms;
    prev_time_boot_ms = time_boot_ms;

    WriteLogFile(roll, yaw, pitch);
}

// ФУНКЦИЯ КОНТРОЛЯ СОСТОЯНИЯ СИСТЕМЫ (БАТАРЕИ).
void Stabilize::SysStatusSlot (int onboard_control_sensors_present, int onboard_control_sensors_enabled, int onboard_control_sensors_health, int load, float voltage_battery, float current_battery, int drop_rate_comm, int errors_comm, int errors_count1, int errors_count2, int errors_count3, int errors_count4, float battery_remaining)
{
0;
}

// ФУНКЦИЯ ПРИЕМА СОСТОЯНИЯ И КООРДИНАТ СНС PIXHAWK.
void Stabilize::RawGPSSlot (float time_usec, double lat, double lon, float alt, float eph, float epv, float vel, float cog, int fix_type, int satellites_visible, float alt_ellipsoid, float h_acc, float v_acc, float vel_acc, float hdg_acc )
{
    pixSNSFix = fix_type;
    pixSNSNumSat = satellites_visible; //СНС Кол-во спутников
}


// ФУНКЦИЯ ПРИЕМА СОСТОЯНИЯ И КООРДИНАТ СНС NMEA (МОКСАН) И РАСЧЁТА ПУТЕВОГО УГЛА.
void Stabilize::NMEArecieved (double NMEALat, double NMEALon, float NMEAHDop, int NMEANsat)
{
    NmeaLatPrevm = NmeaLatm;
    NmeaLonPrevm = NmeaLonm;
    NmeaLat = NMEALat;
    NmeaLon = NMEALon;
    poTransformWgs84ToSample->Transform(1, &NmeaLon, &NmeaLat);
    NmeaLatm = NmeaLat;
    NmeaLonm = NmeaLon;
    NmeaHdop = NMEAHDop;
    NmeaNsat = NMEANsat;

    //Путевой угол по СНС
    deltaNmeaLatm = NmeaLatm - NmeaLatPrevm;
    deltaNmeaLonm = NmeaLonm - NmeaLonPrevm;
    NmeaPsi = atan2(deltaNmeaLonm, deltaNmeaLatm);

    //Осреднение путевого угла NMEA
    for (int i = 3; i>0; i--)
    {
        NP[i] = NP[i-1];
    }
        NP[0] = NmeaPsi;

    double NPsum = 0;

    for (int i = 0; i<4; i++)
    {
        NPsum = NPsum + NP[i];
    }
    NmeaPsiMean = NPsum/4;


    //Разница путевых углов между инерциалкой и NMEA
    deltaNmeaPsi = NmeaPsiMean - pixGPPsiMean;

    if (((pixSNSNumSat < 4) && (pixSNSFix < 2)) || parameters.useUBX == false) //Когда UBX не работает, обновляем расчетную (текущую) координату по Nmea.
    {
        navLatm = NmeaLatm;
        navLonm = NmeaLonm;
    }
}

// ФУНКЦИЯ ПРИЕМА СОСТОЯНИЯ И КООРДИНАТ ИНС PIXHAWK И РАСЧЁТА ПУТЕВОГО УГЛА.
void Stabilize::GlobalPositionSlot ( float time_boot_ms, double lat, double lon, float alt, float relative_alt, float vx, float vy, float vz, float hdg )
{
 ReadCfg(); //Читаем параметры из файла[FOR MODEL]
    navtimeMsReal = time_boot_ms; //Записываем время в глобальную переменную. Время понадобится, например, для расчета путевой скорости.
    pixGPLatPrevm = pixGPLatm;
    pixGPLonPrevm = pixGPLonm;
    //poTransformWgs84ToSample->Transform(1, &pixGPLonPrevm, &pixGPLatPrevm);
//    pixGPLat = lat/10000000; //Инерциальная широта
//    pixGPLon = lon/10000000; //Инерциальная долгота
    pixGPLat = lat; //Инерциальная широта
    pixGPLon = lon; //Инерциальная долгота
    pixGPLatm = pixGPLat;
    pixGPLonm = pixGPLon;
    poTransformWgs84ToSample->Transform(1, &pixGPLonm, &pixGPLatm);
    pixGPhdg = hdg; //Инерциальный курс
    qDebug() << pixGPLat << pixGPLon;

    navTrackAngle = calcRealCoords(navtimeMsReal, navtimeMsPrev); //Вызов функции расчёта текущих (поправленных) координат и текущего (поправленного) путевого угла


    //qDebug() << pixGPLatm << pixGPLonm << navTrackAngle << DesTrackAngle << pixSNSLat << pixGPLat << pixSNSFix;

    if ((TakeOffIsChecked == 0) && parameters.throttleArm && parameters.takeOff) {
        ModeTakeOff();
    }

    if ((TakeOffIsChecked == 1) && parameters.throttleArm && parameters.takeOff ) //Если взлёт зачтён, то ВЫЗЫВАЕМ ФУНКЦИЮ РАСЧЁТА ЗАДАННОГО КРЕНА И ФУНКЦИЮ РАСЧЁТА ЗАДАННОГО ТАНГАЖА.
    {
        DesAlt = calcDesAlt(WP, Letap, R);
        DesV = calcDesSPD(pidV, WP, DesV);
        ThrottlePWM = calcDesThrottlePitch(VTT_PID, VTT_Error, arspd, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesV) + calcDesThrottlePitch(ATT_PID, ATT_Error, hudBaroAlt, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesAlt);
        DesPitch = calcDesThrottlePitch(VTP_PID, VTP_Error, arspd, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesV) + calcDesThrottlePitch(ATP_PID, ATP_Error, hudBaroAlt, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesAlt);
        qDebug() << "DesPitch GP" << DesPitch << "Des Alt GP" << DesAlt << "Des V GP" << DesV<< "VTP" << calcDesThrottlePitch(VTP_PID, VTP_Error, arspd, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesV) << "ATP" <<calcDesThrottlePitch(ATP_PID, ATP_Error, hudBaroAlt, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesAlt);
        //      DesTrackAngle = calcZPU( ZPU_PID, ZPU_Error, WP, Navkoeff, arspd, GS, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, Letap );

//        if (WP.actualWP == (WP.numWP)) //Если номер ППМ, на который выполняется полёт, превышает общее количество ППМ (то есть следующего ППМ нет), то зафиксировать путевой угол, и начать процедуру посадки.
//            {
//                PlaneLand(WP);
//                }
        if (((WP.PointType.at(WP.prevWP) == 208) || (WP.PointType.at(WP.prevWP - 1) == 208) || parameters.doParachuteLand)) //Процедура выброса парашюта
        {
            parameters.doParachuteLand = true;
            DeployParachute();
        }
        if(WP.PointType.at(WP.actualWP) == 21 ||
           WP.PointType.at(WP.actualWP - 1) == 21 ||
           WP.actualWP == WP.numWP)  //Если тип waypoint - посадка, то заданная скорость - посадочная, газ выключен.
        {
            PlaneLand(WP);
        }
        else
        {
         DesTrackAngle = calcZPU( ZPU_PID, ZPU_Error, WP, Navkoeff, arspd, GS, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, Letap );
        }
        DesRoll = calcDesRoll ( YTR_PID, YTR_Error, arspd, GS, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, navTrackAngle, DesTrackAngle);
    qDebug() << "Des Pitch GP Slot" << DesPitch << "Des Alt GP Slot" << DesAlt << "Des V GP Slot" << DesV;
    }

    if (!parameters.throttleArm && !parameters.takeOff) {
        //ThrottlePWM = 1;
        ThrottlePWM = -1;//FOR MODEL
        AileronPWM = 0;
        ElevatorPWM = 0;
        RudderPWM = 0;
    }

    navtimeMsPrePrev = navtimeMsPrev;
    navtimeMsPrev = navtimeMsReal;

    //__________________________
    //TEST
    //qDebug() << TakeOffIsChecked << "\t"  << DesV<< "\t" << arspd << "\t" << DesPitch << ElevatorPWM <<"\t" << hudBaroAlt << "\t" << ThrottlePWM << ATT_Error.Integral << VTT_Error.Integral << DesAlt << Letap << R;
    //__________________________
}


//ФУНКЦИЯ ПРИЁМА ПАРАМЕТРОВ ИНТЕРФЕЙСА ПИЛОТА (ВОЗДУШНАЯ СКОРОСТЬ, СКОРОПОДЪЕМНОСТЬ, КУРС, ВЫСОТА)
void Stabilize::VfrHudSlot ( float airspeed, float groundspeed, float alt, float climb, float heading, float throttle )
{
    if (parameters.modeTest == true) {
        arspd = testV; //Use defined airspeed for the test mode
        hudBaroAlt = testAlt; //Use defined airspeed for the test mode
        qDebug() << "test";
    }
    if (parameters.modeTest != true) {
        arspd = airspeed; //Воздушная скорость
        hudBaroAlt = alt; //Баровысота
        qDebug() << "no test";
    }
    hudGSpeed = groundspeed; //Путевая скорость из интерфейса
    hudHdg = heading; //Магнитный курс

}

//ФУНКЦИЯ РАСЧЁТА ПИД СТАБИЛИЗАЦИИ. ВОЗВРАЩАЕТ ШИМ. Ниже её перегруженная копия для расчета шим элеронов и руля направления.




//ФУНКЦИЯ РАСЧЁТА ИСПРАВЛЕННЫХ КООРДИНАТ И ПУТЕВОГО УГЛА. ВОЗВРАЩАЕТ КООРДИНАТЫ, ПУТЕВОЙ УГОЛ, ПУТЕВУЮ СКОРОСТЬ.
double Stabilize::calcRealCoords ( float navtimeMsReal, float navtimeMsPrev )
{
   // qDebug()<<"calcRealCoords";
    navLatmPrev = navLatm;
    navLonmPrev = navLonm;

    if ((pixSNSNumSat > 4) & (pixSNSFix >= 2))
    {
        navLatm = pixGPLatm;
        navLonm = pixGPLonm;
        navTrackAngle = pixGPPsiMean;
       // qDebug()<<"inert";
    }

    //Путевой угол по инерциалке
    double deltapixGPLatm = pixGPLatm - pixGPLatPrevm;
    double deltapixGPLonm = pixGPLonm - pixGPLonPrevm;
    pixGPPsi = atan2(deltapixGPLonm, deltapixGPLatm);

    for (int i = 3; i>0; i--)
    {
        GPP[i] = GPP[i-1];
    }
    GPP[0] = pixGPPsi;

    double GPPsum = 0;

    for (int i = 0; i<4; i++)
    {
        GPPsum += GPP[i];
    }
    pixGPPsiMean = GPPsum/4;


    if ((pixSNSNumSat > 4) & (pixSNSFix >= 2))
    {
    navTrackAngle = pixGPPsiMean;
    }

    if ((pixSNSNumSat < 4) || (pixSNSFix < 2))
    {
    navTrackAngle = pixGPPsiMean + deltaNmeaPsi;
    }

    //перемещение по инерциалке за время отсчета

    double GPdeltaL =  (pow(deltapixGPLatm, 2) + pow(deltapixGPLonm, 2));

    GPdeltaL = pow(GPdeltaL, 0.5);
    deltaBm = GPdeltaL * sin(navTrackAngle);
    deltaLm = GPdeltaL * cos(navTrackAngle);

    if ((pixSNSNumSat < 4) || (pixSNSFix < 2))
    {
        navLatm = navLatm + deltaBm;
        navLonm = navLonm + deltaLm;
    }

    //Путевая скорость по навигационным координатам
    double navdeltaBm = navLatm - navLatmPrev;
    double navdeltaLm = navLonm - navLonmPrev;
    double navdeltaL =  (pow(navdeltaBm, 2) + pow(navdeltaLm, 2));
    navdeltaL = pow(navdeltaL, 0.5);

    GS = navdeltaL / ((navtimeMsReal - navtimeMsPrev)/1000); //Это понятно: смещение координат делим на время, за которое это смещение произошло

    for (int i = 6; i>0; i--)                                       //ЧТО ЗДЕСЬ НАХУЙ ТАКОЕ ПРОИСХОДИТ?????
    {                                                               //Ах ты грязный извращенец! Это такое скользящее среднее, пиздец нах.
        GSM[i] = GSM[i-1];
    }
    GSM[0] = GS;

    double GSsum = 0;

    for (int i = 0; i<7; i++)
    {
        GSsum += GSM[i];
    }
    GS = GSsum/7;       //А!! Ебать, да это же осреднение по семи точкам! Охуеть, ебанутца лапти гнутца.


//    qDebug() << QString("%1 %2 %3 %4 %5 %6")
//                .arg(pixGPLatm, 0,'f',2)
//                .arg(pixGPLonm, 0,'f',2)
//                .arg(NmeaLatm, 0,'f',2)
//                .arg(NmeaLonm, 0,'f',2)
//                .arg(navLatm, 0,'f',2)
//                .arg(navLonm, 0,'f',2)  <<pixGPPsiMean*180/3.1415926 << NmeaPsiMean*180/3.1415926 << navTrackAngle*180/3.1415926 << GS << AileronPWM << rRoll*180/3.1415 << DesRoll*180/3.1415;



return navTrackAngle;
}

//ФУНКЦИЯ РАСЧЁТА ЗАДАННОГО КРЕНА. ВОЗВРАЩАЕТ ЗАДАННЫЙ КРЕН.
float Stabilize::calcDesRoll (PIDKoeff pidKoeff, ERROR &error, float V, float GS, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, double navTrackAngle, double DesTrackAngle)
{
    error.Error = DesTrackAngle - navTrackAngle;
    error.Error = ReduceAngle(error.Error);
    error.Error = qBound(pidKoeff.LimErrorMin, error.Error, pidKoeff.LimErrorMax);
    //error.Error = (-1) * error.Error;
    PropPart = pidKoeff.Kp * error.Error;
    error.Integral = error.IntegralPrev+(((error.ErrorPrev + error.Error)/2)*((navtimeMsReal - navtimeMsPrev)/1000));
    error.Integral = qBound(pidKoeff.LimIntMin, error.Integral, pidKoeff.LimIntMax);
    if ((TakeOffIsChecked != 1) || (relativeTime < 5000)) //Интрегал отключен до зачёта взлёта
    {
        error.Integral = 0;
        error.IntegralPrev = 0;
    }
    IntPart = pidKoeff.Kp * pidKoeff.Ki * error.Integral;
    //IntPart = qBound(pidKoeff.LimIntMin, IntPart, pidKoeff.LimIntMax);
    error.Differential =  (((error.Error + error.ErrorPrev) / 2 ) - error.ErrorPrePrev) /  ((navtimeMsReal - navtimeMsPrePrev)/1000);
    DifPart = pidKoeff.Kp * pidKoeff.Kd *  error.Differential;
    float YTR_DesRoll = (PropPart + IntPart + DifPart + (pow (error.Error, 7)) ) * (0.2 + (pow ((GS / ( V ? V : 0.05) ), 2)));
    YTR_DesRoll = qBound(pidKoeff.LimPWMMin, YTR_DesRoll, pidKoeff.LimPWMMax);

    error.ErrorPrePrev = error.ErrorPrev;
    error.ErrorPrev = error.Error;
    error.IntegralPrev = error.Integral;

    if ((TakeOffIsChecked != 1) || (relativeTime < 5000)) //Более того, пока не зачтён взлёт, заданный угол крена равен нулю.
    {
        YTR_DesRoll = 0;
    }

    return YTR_DesRoll;
}


//ФУНКЦИЯ РАСЧЁТА ЗАДАННОГО ФАКТИЧЕСКОГО ПУТЕВОГО УГЛА (ЗФПУ)
double Stabilize::calcZPU (PIDKoeff pidKoeff, ERROR &error, WAYPOINTS &WP, NAVKOEFF Navkoeff, float V, float GS, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, double &Letap )
{
    DesTrackAnglePrev = DesTrackAngle; //Сохраняем предыдущий

    // Заданный курс маршрутной линии текущего этапа (ЗПУ)
    double deltaY1Y2 = ((WP.WPLon.at(WP.actualWP))-(WP.WPLon.at(WP.actualWP - 1)));
    double deltaX1X2 = ((WP.WPLat.at(WP.actualWP))-(WP.WPLat.at(WP.actualWP - 1)));
    double PsiM = atan2(deltaY1Y2, deltaX1X2);

    //Координаты точки ЛУР, принадлежащие маршрутной линии текущего этапа
    float deltaLur = Navkoeff.RLur * (GS/V);
    deltaLur = qBound(Navkoeff.RLurMin, deltaLur, Navkoeff.RLurMax);

    //Курс на точку ЛУР

    float PsiPPM = qAtan2((navLonm - (WP.WPLon.at(WP.actualWP))), (navLatm - ((WP.WPLat.at(WP.actualWP))))) + M_PI; //Текущий курс на текущий ППМ
    PsiPPM = ReduceAngle(PsiPPM);
//    if (PsiPPM < (-1 * M_PI))
//    {
//        PsiPPM = PsiPPM + (2*M_PI);
//    }
//    if (PsiPPM > M_PI)
//    {
//        PsiPPM = PsiPPM - (2*M_PI);
//    }
    Lppm = qSqrt(pow((navLonm - (WP.WPLon.at(WP.actualWP))),2) + pow((navLatm - (WP.WPLat.at(WP.actualWP))),2)); //Расстояние по прямой до текущего ППМ

float deltaPsi = PsiPPM - navTrackAngle;
deltaPsi = ReduceAngle(deltaPsi);

//if (deltaPsi < (-1 * M_PI))
//{
//    deltaPsi = deltaPsi + (2*M_PI);
//}
//if (deltaPsi > M_PI)
//{
//    deltaPsi = deltaPsi - (2*M_PI);
//}

qDebug() << "PsiPPM" << PsiPPM << "PsiM" << PsiM;
    //Расчет ЛБУ

    double part_deltaWP = (pow(deltaY1Y2, 2) + pow(deltaX1X2, 2));
    double deltaWP = qSqrt( part_deltaWP ? part_deltaWP : 0.005 ); //Расстояние между текущим и предыдущим ППМ
    Letap = deltaWP; //Передача длины этапа для последующего расчета заданной высоты
    error.Error = (deltaY1Y2 * navLatm - deltaX1X2*navLonm + (WP.WPLat.at(WP.actualWP)) * (WP.WPLon.at(WP.actualWP - 1)) - (WP.WPLon.at(WP.actualWP))*(WP.WPLat.at(WP.actualWP - 1))) / deltaWP; //ЛБУ
qDebug() << "WP" << WP.actualWP;
    WP.cosPsiTs = error.Error / Lppm; //Косинус угла между траверзом и курсом на текущий ППМ
    R = Lppm * sin (acos (WP.cosPsiTs)); //Фактическое расстояние до ППМ по проекции на линию заданного маршрута

    CheckWP(WP, R, deltaLur, Lppm); //Проверка зачёта ППМ
    //Условия зачёта ППМ

     //_______________________________
     //TEST
    // qDebug() << PsiM*180/3.14 << PsiPPM*180/3.14 << Lppm << R << deltaLur << error.Error << DesRoll*180/3.14;
      //qDebug()  << R << deltaLur << DesRoll*180/3.14 << WP.actualWP;
     //_______________________________

  //  if (WP.actualWP < WP.numWP) //Если номер ППМ, на который выполняется полёт, не превышает общего количества ППМ, то продолжать полёт.
   // {
    error.Error = qBound(pidKoeff.LimErrorMin, error.Error, pidKoeff.LimErrorMax); //Для расчётов ЛБУ ограничивается заданными пределами

    PropPart = pidKoeff.Kp * error.Error;

    error.Integral = error.IntegralPrev+(((error.ErrorPrev + error.Error)/2)*((navtimeMsReal - navtimeMsPrev)/1000));
    error.Integral = qBound(pidKoeff.LimIntMin, error.Integral, pidKoeff.LimIntMax);

    if ((TakeOffIsChecked != 1) || (relativeTime < 5000) || (WP.prevWP != WP.actualWP) )
    {
        error.Integral = 0;
        error.IntegralPrev = 0;
    }

    IntPart = pidKoeff.Kp * pidKoeff.Ki * error.Integral;

    error.Differential =  (((error.Error + error.ErrorPrev) / 2 ) - error.ErrorPrePrev) /  ((navtimeMsReal - navtimeMsPrePrev)/1000);

    DifPart = pidKoeff.Kp * pidKoeff.Kd *  error.Differential;

    DesTrackAngle = PsiPPM + PropPart * cos(deltaPsi) + IntPart + DifPart;
    DesTrackAngle = ReduceAngle(DesTrackAngle);
    qDebug() << "PsiM" << PsiM*180/M_PI << "PsiPPM" << PsiPPM*180/M_PI << "NTA" <<navTrackAngle*180/M_PI << "DTA" << DesTrackAngle*180/M_PI << "DeltaPsi" << deltaPsi*180/M_PI <<"cosDeltaPsi" << cos(deltaPsi);

   // }

//    if (WP.actualWP == WP.numWP) //Если номер ППМ, на который выполняется полёт, превышает общее количество ППМ (то есть следующего ППМ нет), то зафиксировать путевой угол, и начать процедуру посадки.
//    {
//    DesTrackAngle = DesTrackAnglePrev;
//    }

    error.ErrorPrePrev = error.ErrorPrev;
    error.ErrorPrev = error.Error;
    error.IntegralPrev = error.Integral;
    //______________________________
    //TEST
   // qDebug() << navTrackAngle*180/3.14 << DesTrackAngle*180/3.14 << WP.actualWP;
    //___________________________________________

    return DesTrackAngle;
}

//ФУНКЦИЯ РАСЧЕТА ЗАДАННОЙ ВЫСОТЫ

float Stabilize::calcDesAlt(WAYPOINTS WP, float Letap, float R)
{
    float DesAlt = (WP.ParamAlt.at(WP.actualWP)) - ((((WP.ParamAlt.at(WP.actualWP))-(WP.ParamAlt.at(WP.actualWP - 1))) / Letap) * R);
    DesAlt = qBound(qMin((WP.ParamAlt.at(WP.actualWP)), (WP.ParamAlt.at(WP.actualWP - 1))), DesAlt, qMax((WP.ParamAlt.at(WP.actualWP)), (WP.ParamAlt.at(WP.actualWP - 1)))); //Загоняем желаемую высоту в коридор не ниже нижней, не выше верхней
    return DesAlt;
}


//ФУНКЦИЯ РАСЧЕТА ЗАДАННОЙ СКОРОСТИ
float Stabilize::calcDesSPD(PID_V pidV, WAYPOINTS WP, float &DesV)
{
float DesSPD = DesV;
    if (WP.PointType.at(WP.actualWP) == 178)
    {
        DesSPD = (WP.ParamSPD.at(WP.actualWP));
    }
    if (DesSPD < pidV.Vmin)
    {
        DesSPD = pidV.Vmin;
    }
    if (DesSPD > pidV.Vmax)
    {
        DesSPD = pidV.Vmax;
    }
    DesV = DesSPD;

    return DesSPD;
}

void Stabilize::SetDesV(float newDesV){

    DesV = newDesV;

    if (DesV < pidV.Vmin)
    {
        DesV = pidV.Vmin;
    }
    if (DesV > pidV.Vmax)
    {
        DesV = pidV.Vmax;
    }
    qDebug() << "New Airspeed" << DesV;
}

//ФУНКЦИЯ РАСЧЕТА ЗАДАННОГО ТАНГАЖА


float Stabilize::calcDesThrottlePitch(PIDKoeff pidKoeff, ERROR &error, float Value, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, float calculatedValue) {
    //(PIDKoeff pidKoeff, ERROR &error, float Value, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, float calculatedValue)

    //СКОРОСТЬ В ТАНГАЖ

    error.Error = calculatedValue - Value;
    error.Error = qBound(pidKoeff.LimErrorMin, error.Error, pidKoeff.LimErrorMax);

    //error.Error = (-1) * error.Error;

    float PropPart = pidKoeff.Kp * error.Error;
    error.Integral = error.IntegralPrev+(((error.ErrorPrev + error.Error)/2)*((navtimeMsReal - navtimeMsPrev)/1000));
    error.Integral = qBound(pidKoeff.LimIntMin, error.Integral, pidKoeff.LimIntMax);
    float IntPart = pidKoeff.Kp * pidKoeff.Ki * error.Integral;
   // IntPart = qBound(pidKoeff.LimIntMin, IntPart, pidKoeff.LimIntMax);

    error.Differential =  (((error.Error + error.ErrorPrev) / 2 ) - error.ErrorPrePrev) /  ((navtimeMsReal - navtimeMsPrePrev)/1000);
    float DifPart = pidKoeff.Kp * pidKoeff.Kd *  error.Differential;
    float desValue = (-1) * (PropPart + IntPart + DifPart);
    desValue = qBound(pidKoeff.LimPWMMin, desValue, pidKoeff.LimPWMMax) * pidKoeff.Kv;

    error.ErrorPrePrev = error.ErrorPrev;
    error.ErrorPrev = error.Error;
    error.IntegralPrev = error.Integral;

    return desValue;
}

int Stabilize::calcPWM(PWMLimits pwmLimits, float inPWM) {
    if (pwmLimits.inverse == true) {
        inPWM = (-1) * inPWM;
    }

    int PWM = ((pwmLimits.maxpwm - pwmLimits.minpwm) / 2) * inPWM + pwmLimits.trim;
    PWM = qBound(pwmLimits.minpwm, PWM, pwmLimits.maxpwm);
    return PWM;
}

//Процедура чтения и преобразования координат из WP файла.

void Stabilize::ReadWaypoints(void)
{
    file1.setFileName(QCoreApplication::applicationDirPath() + "/wp.waypoints");
    file1.open(QFile::ReadOnly);
    if(file1.isOpen())
    {
       QTextStream wp(&file1);
       WP.Param1.clear();
       wp.readLine();
       while (!wp.atEnd())
       {
           QStringList wpstrl = wp.readLine().split("\t");
           WP.WPnum.append(wpstrl.at(0).toInt());
           WP.Param1.append(wpstrl.at(1).toInt());
           WP.Param2.append(wpstrl.at(2).toInt());
           WP.PointType.append(wpstrl.at(3).toInt());
           WP.Param4.append(wpstrl.at(4).toFloat());
           WP.ParamSPD.append(wpstrl.at(5).toFloat());
           WP.Param6.append(wpstrl.at(6).toFloat());
           WP.Param7.append(wpstrl.at(7).toFloat());
           WP.WPLat.append(wpstrl.at(8).toDouble());
           WP.WPLon.append(wpstrl.at(9).toDouble());
           WP.ParamAlt.append(wpstrl.at(10).toFloat());
           WP.Param11.append(wpstrl.at(11).toInt());
       }
//Чтобы избежать вылета всего алгоритма при зачёте последнего ППМ, добавляем в конце полетного задания некий виртуальный ППМ, вблизи которого мы, вероятно, никогда не окажемся, и курс на который мы никогда не высчитываем.
       WP.WPnum.append(WP.WPnum.last()+1);
       WP.Param1.append(WP.WPnum.last());
       WP.Param2.append(WP.WPnum.last());
       WP.PointType.append(16);
       WP.Param4.append(0);
       WP.ParamSPD.append(pidV.Vland);
       WP.Param6.append(0);
       WP.Param7.append(0);
       WP.WPLat.append(56.768);
       WP.WPLon.append(38.770);
       WP.ParamAlt.append(0);
       WP.Param11.append(1);

       WP.numWP = WP.WPnum.count();
       //qDebug() <<wp8.at(0) << wp1.count();
       //qDebug() << QString("%1").arg(WP.wp8.at(0), 0,'f',8) << WP.numWP;
    }
    else qDebug() << "Файл полетного задания не найден!";
    file1.close();

    //__________________________________________________
    //FOR TEST
    qDebug() << "Flight plan coordinates in WGS";
    for (int i = 0; i < WP.numWP; i++) {
        qDebug() <<i << WP.WPLon.at(i) << WP.WPLat.at(i);
    }
    //________________________________________________

     qDebug() << "Flight plan coordinates in meters";
    for (int i = 0; i < WP.numWP; i++)
    {

        poTransformWgs84ToSample->Transform(1, &WP.WPLon[i], &WP.WPLat[i]);

        //__________________________________________________
        //FOR TEST
        qDebug() <<i << WP.WPLon.at(i) << WP.WPLat.at(i);
        //________________________________________________
    }
}


void Stabilize::WriteLogFile(float roll, float yaw, float pitch) {
    if(file.isOpen())
    {
        QTextStream stream(&file);
        stream
                << QString("%1").arg(pixGPLonm , 0, 'f', 2) << "\t"
                << QString("%1").arg(pixGPLatm , 0, 'f', 2) << "\t"
                << QString("%1").arg(NmeaLonm , 0, 'f', 2) << "\t"
                << QString("%1").arg(NmeaLatm , 0, 'f', 2) << "\t"
                << QString("%1").arg(navLonm , 0, 'f', 2) << "\t"
                << QString("%1").arg(navLatm , 0, 'f', 2) << "\t"
                << pixGPPsi << "\t"
                << pixGPPsiMean << "\t"
                << NmeaPsi << "\t"
                << NmeaPsiMean << "\t"
                << WP.actualWP << "\t"
                << navTrackAngle << "\t"
                << DesTrackAngle << "\t"
                << GS << "\t"
                << roll << "\t"
                << DesRoll << "\t"
                << AileronPWMOut << "\t"
                << pitch << "\t"
                << DesPitch << "\t"
                << ElevatorPWMOut << "\t"
                << yaw << "\t"
                << DesTrackAngle << "\t"
                << RudderPWMOut << "\t"
                << arspd << "\t"
                << DesV << "\t"
                << ThrottlePWMOut << "\t"
                << hudBaroAlt << "\t"
                << DesAlt << "\n";
      }
}


//Функция преобразования координат
void Stabilize::createCoordinateTransformation(const QString &prjFileName)
{
    GDALAllRegister();

    poTransformWgs84ToSample = poTransformSampleToWgs84 = NULL;
    QFile prjFile(prjFileName);
    if(prjFile.open(QFile::ReadOnly))
    {
        OGRSpatialReference *spWGS84 = new OGRSpatialReference();
        spWGS84->SetWellKnownGeogCS("WGS84");
        OGRSpatialReference *spSample = new OGRSpatialReference(prjFile.readAll().data());
        prjFile.close();

        poTransformWgs84ToSample = OGRCreateCoordinateTransformation(spWGS84, spSample);
        poTransformSampleToWgs84 = OGRCreateCoordinateTransformation(spSample, spWGS84);
    }
    else
    {
        qDebug() << __FUNCTION__ << prjFileName;
    }
}

void Stabilize::CheckWP(WAYPOINTS &WP, float R, float deltaLur, float Lppm) {
    if ((qAbs(R) <= qAbs(deltaLur)) && (Lppm <= 1.5 * deltaLur)) //Если фактическое расстояние до ППМ по проекции на линию заданного маршрута меньше, чем ЛУР, то происходит зачёт
   {
       WP.prevWP = WP.actualWP; //Текущая точка становится предыдущей
       WP.actualWP ++;          //Номер ППМ увеличивается
       qDebug() << "Waypoint checked:" << WP.prevWP << "Next waypoint:" << WP.actualWP;
   }

}

float Stabilize:: ReduceAngle(float angle) {
    if (angle < (-1 * M_PI))
    {
        angle = angle + (2*M_PI);
    }
    if (angle > M_PI)
    {
        angle = angle - (2*M_PI);
    }
    return angle;
}

void Stabilize::ModeTakeOff() {

    if ((TakeOffIsChecked == 0) && parameters.throttleArm && parameters.takeOff) //Если взлёт не зачтён, но разрешен, то крен нужно держать нулевым, а тангаж - взлётным, газ - максимальный.
    {
        DesRoll = 0;
        DesPitch = PitchPID.ElevTkoffPitch;
//        ThrottlePWM = -1; //Максимальный газ
         ThrottlePWM = 2; //Максимальный газ [FOR MODEL]

         DesAlt = TkOffAltCheck;
    qDebug() << "Mode Takeoff";
    if (hudBaroAlt >= TkOffAltCheck)
    {
        TakeOffIsChecked = 1;
        qDebug() << "Takeoff is checked!";
    }
    }
}

void Stabilize::ModeWPNav() {
    0;
}

void Stabilize::DeployParachute() {
    if (deployParachuteCount < deployParachuteReopen) {
        qDebug() << "Parachute";
  // ThrottlePWM = 1;
    ThrottlePWM = -1;//[FOR MODEL]
   if (deployParachuteTime == 0) {
    deployParachuteTime = navtimeMsReal;
    deployParachuteSPD = arspd;
   }
   if ((navtimeMsReal >= deployParachuteTime + deployParachuteDelay) && (navtimeMsReal < (deployParachuteTime + deployParachuteDelay + deployParachuteDuration ))) {
       ParachutePWM = 1;
        qDebug() << "Parachute deployed";
   }
   if (navtimeMsReal >= (deployParachuteTime + deployParachuteDelay + deployParachuteDuration )) {
       ParachutePWM = -1;
        qDebug() << "Parachute closed";
   }
   if (navtimeMsReal >= (deployParachuteTime + 2*deployParachuteDelay + deployParachuteDuration ) && (arspd >= (deployParachuteSPD - deployParachuteReopenSPD))) {
       deployParachuteTime = 0;
       deployParachuteCount++;
        qDebug() << "Parachute redeployed" << deployParachuteCount;
   }
  }
    else {
        PlaneLand(true);
    }
}

void Stabilize::PlaneLand(WAYPOINTS WP) {

    if (WP.actualWP == (WP.numWP)) //Если посадка по окончании маршрута (по зачёту самой последней точки)
    {
        DesTrackAngle = DesTrackAnglePrev;
        DesV = pidV.Vland;
    //            ThrottlePWM = 1;
        ThrottlePWM = -1;//[FOR MODEL]
        DesAlt = 3;
    }

    if ((WP.PointType.at(WP.actualWP) == 21)||(WP.PointType.at(WP.actualWP - 1) == 21))  //Если тип waypoint - посадка, то заданная скорость - посадочная, газ выключен.
    {
        qDebug() << "Land begin" << Lppm;
        if (hudBaroAlt > (WP.ParamAlt.at(WP.actualWP))+ landThrottleCutoffAlt) {
            DesTrackAngle = calcZPU( ZPU_PID, ZPU_Error, WP, Navkoeff, arspd, GS, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, Letap );
        qDebug() << "Land, calc DTA";
        }

        if ((hudBaroAlt < (WP.ParamAlt.at(WP.actualWP) + landThrottleCutoffAlt))&&(hudBaroAlt >= (WP.ParamAlt.at(WP.actualWP) + landFlareAlt))) {
          DesV = pidV.Vland;
          //ThrottlePWM = 1;


          ThrottlePWM = -1;//[FOR MODEL]
          qDebug() << "Land";

       }

       if (hudBaroAlt < (WP.ParamAlt.at(WP.actualWP) + landFlareAlt)) {
          DesPitch = landFlarePitch;
          ThrottlePWM = -1;//[FOR MODEL]
          //DesTrackAngle = DesTrackAnglePrev;
          //DesRoll = 0;
          qDebug() << "Land Flare";

       }

    }
}

void Stabilize::PlaneLand(bool doPlaneLand) {
    if ((parameters.doPlaneLand) || doPlaneLand) {
        qDebug() << "Urgent Plane Land begin" << Lppm;
        if (hudBaroAlt > landThrottleCutoffAlt) {
            DesTrackAngle = calcZPU( ZPU_PID, ZPU_Error, WP, Navkoeff, arspd, GS, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, Letap );
            //ThrottlePWM = 1;
            ThrottlePWM = -1;//[FOR MODEL]
            DesAlt = 1;
            DesV = pidV.Vmin;
            DesPitch = calcDesThrottlePitch(VTP_PID, VTP_Error, arspd, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesV) + calcDesThrottlePitch(ATP_PID, ATP_Error, hudBaroAlt, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesAlt);

        qDebug() << "Urgent Plane Land, calc DTA";
        }

        if ((hudBaroAlt < landThrottleCutoffAlt) && (hudBaroAlt >=  landFlareAlt)) {
          DesV = pidV.Vland;
          //ThrottlePWM = 1;
          ThrottlePWM = -1;//[FOR MODEL]
          DesAlt = 1;
          DesPitch = calcDesThrottlePitch(VTP_PID, VTP_Error, arspd, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesV) + calcDesThrottlePitch(ATP_PID, ATP_Error, hudBaroAlt, navtimeMsReal, navtimeMsPrev, navtimeMsPrePrev, DesAlt);

          qDebug() << "Urgent Plane Land" << DesPitch;

       }

       if (hudBaroAlt < landFlareAlt) {
          DesPitch = landFlarePitch;
          ThrottlePWM = -1;//[FOR MODEL]
          DesAlt = 1;
          //DesTrackAngle = DesTrackAnglePrev;
          //DesRoll = 0;
          qDebug() << "Urgent Plane Land Flare";

       }
    }
}

void Stabilize::ModeAbnormalLand() {
    0;
}

        void Stabilize::SetPitchPidKp(double Kp) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID1/Kp", Kp);
            settings.sync();
            this->PitchPID.Kp = Kp;
        qDebug() << "Kp" <<Kp;
        }
        void Stabilize::SetPitchPidKi(double Ki) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID1/Ki", Ki);
            settings.sync();
            this->PitchPID.Ki = Ki;
        qDebug() << Ki;
        }
        void Stabilize::SetPitchPidKd(double Kd) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID1/Kd", Kd);
            settings.sync();
            this->PitchPID.Kd = Kd;
        qDebug() << Kd;
        }
        void Stabilize::SetPitchPidKOmega(double KOmega) {

            this->PitchPID.KOmega = KOmega;
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID1/Kpd", PitchPID.KOmega);
            settings.sync();
        qDebug() << KOmega;
        }
        void Stabilize::SetPitchPidKElevTurn(double KElevTurn) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/KTurn", KElevTurn);
            settings.sync();
            this->PitchPID.KElevTurn = KElevTurn;
        qDebug() << KElevTurn;
        }
        void Stabilize::SetPitchPidLimErrorMin(double LimErrorMin) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimErrorMin", LimErrorMin);
            settings.sync();
            this->PitchPID.LimErrorMin = LimErrorMin;
        qDebug() << LimErrorMin;
        }
        void Stabilize::SetPitchPidLimErrorMax(double LimErrorMax) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimErrorMax", LimErrorMax);
            settings.sync();
            this->PitchPID.LimErrorMax = LimErrorMax;
        qDebug() << LimErrorMax;
        }
        void Stabilize::SetPitchPidLimPWMMin(double LimPWMMin) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimPWMMin", LimPWMMin);
            settings.sync();
            this->PitchPID.LimPWMMin = LimPWMMin;
        qDebug() << LimPWMMin;
        }
        void Stabilize::SetPitchPidLimPWMMax(double LimPWMMax) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimPWMMax", LimPWMMax);
            settings.sync();
            this->PitchPID.LimPWMMax = LimPWMMax;
        qDebug() << LimPWMMax;
        }
        void Stabilize::SetPitchPidLimIntMin(double LimIntMin) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimIntMin", LimIntMin);
            settings.sync();
            this->PitchPID.LimIntMin = LimIntMin;
        qDebug() << LimIntMin;
        }
        void Stabilize::SetPitchPidLimIntMax(double LimIntMax) {
            QSettings settings(QCoreApplication::applicationDirPath() + cfgName, QSettings::IniFormat);
            settings.setValue("Stab_parameters_elevatorPID2/LimIntMax", LimIntMax);
            settings.sync();
            this->PitchPID.LimIntMax = LimIntMax;
        qDebug() << LimIntMax;
        }
