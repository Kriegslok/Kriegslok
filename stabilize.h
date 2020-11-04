#ifndef STABILIZE_H
#define STABILIZE_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QFile>
#include <QTimer>
#include <QTime>
#include <QPointF>
#include <QVector>

#include "gdal.h"
#include "ogr_spatialref.h"
#include "structs.h"
#include "calcattpid.h"
#include "readcfg.h"

// MAVLink
#include <../c_library_v1/common/mavlink.h>

// Internal
#include "../common/links/udp_link.h"

#include "uav_communicator_factory.h"
#include "../common/mavlink_communicator.h"
//#include "../common/handlers/heartbeat_handler.h"

#include "../command/service_registry.h"
#include "../command/command_service/command_service.h"

#include "uav_model.h"

namespace domain
{

    class Stabilize : public QObject
    {
        Q_OBJECT

    public:
        explicit Stabilize(QObject *parent = 0);
        ~Stabilize();

        // для MAVlink
//        UavCommunicatorFactory *factory;
//        domain::MavLinkCommunicator* communicator;
//        domain::UdpLink *link;
//        domain::UavModel uavModel;
        // -- //

        PIDKoeff RollPID; //Элероны от крена
    private:
        PIDKoeff PitchPID; //Руль высоты от тангажа
    public:
        PIDKoeff YawPID;   //Руль направления от курса
        AttPidCalcResults RollAttPidCalcResult;
        AttPidCalcResults PitchAttPidCalcResult;
        AttPidCalcResults YawAttPidCalcResult;


        OGRCoordinateTransformation *poTransformWgs84ToSample, *poTransformSampleToWgs84;

        Parameters parameters;

        WAYPOINTS WP;

        NAVKOEFF Navkoeff;
        PIDKoeff YTR_PID;  //Коэффициенты и параметры ПИДа "Крен от путевого угла"
        PIDKoeff ZPU_PID;  //Коэффициенты и параметры ПИДа "Путевой угол от координат"
        PIDKoeff ATP_PID;  //Коэффициенты и параметры ПИДа "Тангаж от высоты"
        PIDKoeff ATT_PID;  //Коэффициенты и параметры ПИДа "Газ от высоты"
        PIDKoeff VTP_PID;  //Коэффициенты и параметры ПИДа "Тангаж от скорости"
        PIDKoeff VTT_PID;  //Коэффициенты и параметры ПИДа "Газ от скорости"

        PID_V pidV;
    //void readFromUAVCfg(PIDKoeff &RollPID, PIDKoeff &PitchPID, PIDKoeff &YawPID, Parameters &parameters, NAVKOEFF &Navkoeff, PIDKoeff &YTR_PID, PIDKoeff &ZPU_PID, PIDKoeff &ATP_PID, PIDKoeff &ATT_PID, PIDKoeff &VTP_PID, PIDKoeff &VTT_PID, PID_V &pidV,  ERROR &RollError, ERROR &PitchError, ERROR &YawError, ERROR &YTR_Error, ERROR &ZPU_Error, ERROR &ATP_Error, ERROR &ATT_Error, ERROR &VTP_Error, ERROR &VTT_Error); //Составляющая газа от высоты);
        ERROR RollError;
        ERROR PitchError;
        ERROR YawError;
        ERROR YTR_Error;
        ERROR ZPU_Error;
        ERROR ATP_Error; //Составляющая тангажа от высоты
        ERROR ATT_Error; //Составляющая газа от высоты
        ERROR VTP_Error; //Составляющая тангажа от скорости
        ERROR VTT_Error; //Составляющая газа от высоты

        float currentPitch;
        float currentRoll;
        float currentYaw;
        float currentDesRoll;
        float currentDesPitch;
        float currentDesYaw;

        uint32_t currentTimeBootMs;
        float currentAltitude;
        float currentHeight;
        float currentGAlt;
        float currentDesAlt;
        float currentAirspeed;
        float currentDesV;
        float currentThrottle;


        QFile file;
        QFile file1;

        PWMLimits aileronPWMLimits;
        PWMLimits elevatorPWMLimits;
        PWMLimits rudderPWMLimits;
        PWMLimits throttlePWMLimits;
        PWMLimits parachutePWMLimits;

    private:


        QString cfgName;
        //Переменные параметры алгоритма
        float prev_time_boot_ms; // Предыдущее значение времени
        float pre_prev_time_boot_ms;
        float relativeTime; // мс, Значение времени с момента запуска программы
        int deployParachuteTime; //Момент времени выброса парашюта
        int deployParachuteDelay;
        int deployParachuteDuration;
        int deployParachuteReopen;
        float deployParachuteReopenSPD;
        int deployParachuteCount;
        int deployParachuteSPD;
        bool deployParachuteEnable;
        bool releaseParachuteEnable;
        float releaseParachuteVSPD;
        float fsBatLevel;
        int fsBatAction;
        float fsVVel;
        bool fsVVelDeployParachute;
        float fsSnsNoFixTime;
        bool fsSnsOperatorDisable;
        int fsSnsAction;
        float fsArspdNotActiveTime;
        bool fsArspdFixThrottleCheck;
        int fsArspdFixThrottle;
        bool fsArspdFixPitchCheck;
        float fsArspdFixPitch;
        int fsArspdAction;

        float testV; // Воздушная скорость
        float testAlt;
        float GS; //Путевая скорость
        float GSM[6];
        //float Error;
        /*
        float RollErrorPrev; //Предыдущее значение ошибки крена
        float RollErrorPrePrev; //Предпредыдущее значение ошибки крена
        float PitchErrorPrev; //Предыдущее значение ошибки крена
        float PitchErrorPrePrev; //Предпредыдущее значение ошибки крена
        */
        float PWM; //ШИМ сигнал на выходе ПИДа
       // float ScaleV; //Масштабный коэффициент элеронов по скорости
        float PropPart;
        float IntPart;
        float DifPart;
        float KompensPart;

public:
        float AileronPWM; //Сигнал отклонения рулевой машинки элеронов
        float ElevatorPWM;
        float RudderPWM;
        float ThrottlePWM;
        float ParachutePWM;
        float ThrottlePWMoutFl;
        int AileronPWMOut, ElevatorPWMOut, RudderPWMOut, ThrottlePWMOut, ParachutePWMOut;

private:
    //    float rRoll;
    //    float rPitch;
        float DesRoll; //Заданный угол крена
        float DesPitch; //Заданный угол тангажа
        float DesYaw; //Заданный угол рыскания
        float DesAlt; //Заданная высота
        float DesV; //Заданная скорость
        float desPitchLimitMax; //Ограничение по заданному тангажу
        float desPitchLimitMin; //Ограничение по заданному тангажу
        float landThrottleCutoffAlt;
        float landFlarePitch;
        float landFlareAlt;



private:
        double DesTrackAngle; //Заданный путевой угол (ЗПУ)
        double DesTrackAnglePrev;//ЗПУ предыдущей итерации
        float DTA;

        int TakeOffIsChecked; //Признак зачёта взлёта
        float TkOffAltCheck;



        //Навигационные переменные

        double pixGPLat; //Инерциальная широта
        double pixGPLon; //Инерциальная долгота
        float pixGPhdg; //Инерциальный курс
        double pixGPLatm;
        double pixGPLonm;
        double pixGPLatPrev;
        double pixGPLonPrev;
        double pixGPLatPrevm;
        double pixGPLonPrevm;
        float pixGPPsi; //Вычисленный путевой угол инерциалки
        double GPP[3];
        double pixGPPsiMean;


        double pixSNSLat; //СНС широта
        double pixSNSLon; //СНС долгота
        float pixSNSVel; //СНС путевая скорость
        int pixSNSFix; //СНС fix
        int pixSNSNumSat; //СНС Кол-во спутников
        double pixSNSLatm;
        double pixSNSLonm;
        double pixSNSLatPrev;
        double pixSNSLonPrev;
        double pixSNSLatPrevm;
        double pixSNSLonPrevm;
        double pixSNSPsi; //Вычисленный путевой угол СНС
        double deltapixSNSPsi;
        int SNSPermit;
        QPointF pixSNSm;



        float photoTime; //time_boot_ms на момент съемки
        int outerPermit;

        double NmeaLat;
        double NmeaLon;
        double NmeaLatm;
        double NmeaLonm;
        double NmeaLatPrevm;
        double NmeaLonPrevm;
        float NmeaHdop;
        int NmeaNsat;
        double deltaNmeaLatm;
        double deltaNmeaLonm;
        double deltaNmeaPsi_active;
        double NmeaPsi; //Вычисленный путевой угол СНС
        double deltaNmeaPsi;
        double NP[3];
        double NmeaPsiMean;

        double navLat; //Поправленное значение широты
        double navLon; //Поправленное значение доглоты
        float navHdg; //Поправленное значение курса
        float navVel; //Поправленное значение скорости
        float navTrackAngle; //Поправленное значение путевого угла
        double navLatm; //Текущая поправленная широта
        double navLonm; //Текущая поправленная долгота
        double navLatmPrev; //Предыдущее значение текущей поправленной широты
        double navLonmPrev; //Предыдущее значение текущей поправленной долготы
        float navtimeMsReal; //Текущий отсчет времени, используемого в нафигации
        float navtimeMsPrev; //Предыдущий отсчет времени, используемого в навигации.
        float navtimeMsPrePrev; //Предпредыдущий отсчет времени, используемого в навигации.

        float arspd; //Воздушная скорость
        float hudGSpeed; //Путевая скорость из интерфейса
        float hudBaroAlt; //Баровысота
        float hudHdg; //Магнитный курс

        double deltapixSNSLatm;
        double deltapixSNSLonm;
        double deltapixSNSPsi_active;
        double deltaouterLatm;
        double deltaouterLonm;
        double deltaouterPsi_active;
        double deltaBm;
        double deltaLm;

        float Lppm; //Расстояние по прямой до текущего ППМ
        float R; //Расстояние до проекции на линию пути (траверза)
        double Letap; //Длина этапа

    signals:
       // void needAttPid (PIDKoeff pidKoeff, PID_V pid_v, ERROR error, float V, float time_boot_ms, float prev_time_boot_ms, float pre_prev_time_boot_ms, float Value1, float Value2, float DesValue1, float Omega);
          //void readFromUAVCfg();
    public slots:
        void SysStatusSlot (int onboard_control_sensors_present, int onboard_control_sensors_enabled, int onboard_control_sensors_health, int load, float voltage_battery, float current_battery, int drop_rate_comm, int errors_comm, int errors_count1, int errors_count2, int errors_count3, int errors_count4, float battery_remaining);
        void RawGPSSlot (float time_usec, double lat, double lon, float alt, float eph, float epv, float vel, float cog, int fix_type, int satellites_visible, float alt_ellipsoid, float h_acc, float v_acc, float vel_acc, float hdg_acc );
        void AttitudeSlot (float time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
        void GlobalPositionSlot ( float time_boot_ms, double lat, double lon, float alt, float relative_alt, float vx, float vy, float vz, float hdg );
        void VfrHudSlot ( float airspeed, float groundspeed, float alt, float climb, float heading, float throttle );
        void NMEArecieved (double NMEALat, double NMEALon, float NMEAHDop, int NMEANsat);
        double calcRealCoords (float navtimeMsReal, float navtimeMsPrev );







    private slots:
        void createCoordinateTransformation(const QString &prjFileName);
        void DeployParachute();
        void CheckWP (WAYPOINTS &WP, float R, float deltaLur, float Lppm);
        float ReduceAngle (float angle);
        void ReadCfg();
        void ModeTakeOff();
        void ModeWPNav();
        void PlaneLand(WAYPOINTS WP);
        void PlaneLand(bool doPlaneLand);
        void ModeAbnormalLand();
        float calcDesRoll (PIDKoeff pidKoeff, ERROR &error, float V, float GS, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, double navTrackAngle, double DesTrackAngle);
        float calcDesAlt (WAYPOINTS WP, float Letap, float R);
        float calcDesSPD (PID_V pidV, WAYPOINTS WP, float &DesV);
        float calcDesThrottlePitch (PIDKoeff pidKoeff, ERROR &error, float Value, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev, float calculatedValue);
        void ReadWaypoints(void);
        double calcZPU (PIDKoeff pidKoeff, ERROR &error, WAYPOINTS &WP, NAVKOEFF Navkoeff, float V, float GS, float navtimeMsReal, float navtimeMsPrev, float navtimeMsPrePrev , double &Letap);
        int calcPWM (PWMLimits pwmLimits, float inPWM);
        void WriteLogFile(float roll, float yaw, float pitch);

    public: // TO DO: СДЕЛАТЬ ФУНКЦИЮ void SetDesV(float desiredDesV); и вызывать ее из command_handler.cpp
            void SetDesV(float newDesV);
            void SetPitchPidKp(double Kp);
            void SetPitchPidKi(double Ki);
            void SetPitchPidKd(double Kd);
            void SetPitchPidKOmega(double KOmega);
            void SetPitchPidKElevTurn(double KElevTurn);
            void SetPitchPidLimErrorMin(double LimErrorMin);
            void SetPitchPidLimErrorMax(double LimErrorMax);
            void SetPitchPidLimPWMMin(double LimPWMMin);
            void SetPitchPidLimPWMMax(double LimPWMMax);
            void SetPitchPidLimIntMin(double LimIntMin);
            void SetPitchPidLimIntMax(double LimIntMax);

    };

};



#endif // STABILIZE_H
