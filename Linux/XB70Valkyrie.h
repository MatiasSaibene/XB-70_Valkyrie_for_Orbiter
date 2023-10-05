#ifndef __XB70VALKYRIE_H
#define __XB70VALKYRIE_H

#define STRICT 1
#include "OrbiterAPI.h"
#include "Orbitersdk.h"
#include "VesselAPI.h"
#include "XB70_mesh_definitions.h"

//Vessel parameters
const double XB70_SIZE = 22.8;  //Mean radius in meters.

const double XB70_EMPTYMASS = 115031; //Empty mass in kg.

const double XB70_FUELMASS = 140000; //Fuel mass in kg.

const double XB70_ISP = 2e4; //Fuel-specific impulse in m/s.

const double XB70_MAXMAINTH = 89e3; //Max main thrust in kN.

const double XB70_AFTERBRNTH = 120e3; //Max afterburner thrust.

const double LANDING_GEAR_OPERATING_SPEED = 0.25;

const VECTOR3 XB70_CS = {199.5443, 585.7, 33.2172};

const double XB70_VLIFT_C = 23.94; //Chord lenght in m.

const double XB70_VLIFT_S = 585.07; //Wing area in m^2.

const double XB70_VLIFT_A = 1.751; //Wing aspect ratio.

const double XB70_HLIFT_C = 5.01;

const double XB70_HLIFT_S = 21.74;

const double XB70_HLIFT_A = 1;


//Define touchdown points
//For gear down
static const int ntdvtx_geardown = 12;
static TOUCHDOWNVTX tdvtx_geardown[ntdvtx_geardown] = {
    
    {_V(-0.0676, -4.9817, 27.4759), 3e6, 3e5, 1.6, 0.1}, //Provisory touchdownpoint.
    //{(TDP_Front_landing_gearLocation), 1e6, 1e5, 1.6, 0.1},  //Real touchdown point.
    {(TDP_Rear_left_landing_gearLocation), 3e6, 3e5, 1.6, 0.1},
    {(TDP_Rear_right_landing_gearLocation), 3e6, 3e5, 1.6, 0.1},

    {(TDP_FrontLocation), 1e7, 1e5, 3.0},
    {(TDP_Rear_leftLocation), 1e7, 1e5, 3.0},
    {(TDP_Rear_rightLocation), 1e7, 1e5},
    {(TDP_Left_wingLocation), 1e7, 1e5, 3.0},
    {(TDP_Right_wing_tdpLocation), 1e7, 1e5, 3.0},
    {(TDP_Right_aileronLocation), 1e7, 1e5, 3.0},
    {(TDP_Left_aileronLocation), 1e7, 1e5, 3.0},
    {(TDP_Front_fuselageLocation), 1e7, 1e5, 3.0},
    {(TDP_pitot_probeLocation), 1e7, 1e5, 3.0},
};

//For gear up
static const int ntdvtx_gearup = 9;
static TOUCHDOWNVTX tdvtx_gearup[ntdvtx_gearup] = {
    {(TDP_FrontLocation), 1e7, 1e5, 3.0},
    {(TDP_Rear_leftLocation), 1e7, 1e5, 3.0},
    {(TDP_Rear_rightLocation), 1e7, 1e5},
    {(TDP_Left_wingLocation), 1e7, 1e5, 3.0},
    {(TDP_Right_wing_tdpLocation), 1e7, 1e5, 3.0},
    {(TDP_Right_aileronLocation), 1e7, 1e5, 3.0},
    {(TDP_Left_aileronLocation), 1e7, 1e5, 3.0},
    {(TDP_Front_fuselageLocation), 1e7, 1e5, 3.0},
    {(TDP_pitot_probeLocation), 1e7, 1e5, 3.0},
};

//XB70 class interface

class XB70: public VESSEL4{
    public:
        enum LandingGearStatus{GEAR_DOWN, GEAR_UP, GEAR_DEPLOYING, GEAR_STOWING} landing_gear_status;
        enum DoorStatus{DOOR_CLOSED, DOOR_OPEN, DOOR_CLOSING, DOOR_OPENING} door_status;

        XB70(OBJHANDLE hVessel, int flightmodel);
        virtual ~XB70();

        void DefineAnimations(void);
        void ActivateLandingGear(LandingGearStatus action);
        void ActivateDoor(DoorStatus actiondoor);
        void SetGearDown(void);
        void CloseDoor(void);
        void UpdateLandingGearAnimation(double);
        void UpdateDoorAnimation(double);
        double UpdateParticleLvl();

        void clbkSetClassCaps(FILEHANDLE cfg)override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs)override;
        void clbkSaveState(FILEHANDLE scn)override;
        void clbkPostStep(double, double, double)override;
        int clbkConsumeBufferedKey(int, bool, char *)override;


        PARTICLESTREAMSPEC canard_contrails;
        double lvl;
    
    private:
        unsigned int anim_landing_gear;
        unsigned int anim_door;
        unsigned int anim_elevatortrim;
        unsigned int anim_laileron;
        unsigned int anim_raileron;
        unsigned int anim_elevator;
        unsigned int anim_canards;
        unsigned int anim_lrudder;
        unsigned int anim_rrudder;

        double landing_gear_proc;
        double door_proc;

        AIRFOILHANDLE hwing;
        CTRLSURFHANDLE hlaileron, hraileron, canards;

};

#endif //!__XB70VALKYRIE_H
