#ifndef __XB70VALKYRIE_H
#define __XB70VALKYRIE_H

#define STRICT 1
#include "OrbiterAPI.h"
#include "Orbitersdk.h"
#include "VesselAPI.h"

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
static const int ntdvtx_geardown = 13;
static TOUCHDOWNVTX tdvtx_geardown 
/*[ntdvtx_geardown] = {
    {_V(-0.0494, -1.6817, 7.4897), 1e6, 1e5,1.6, 0.1},
    {_V(3.5852, -1.6817, -6.7983), 1e6, 1e5, 1.6, 0.1},
    {_V(-3.4148, -1.6817, -6.7983), 1e6, 1e5, 1.6, 0.1},
};*/

[ntdvtx_geardown] = {
    {_V(-0.0676, -4.9817, 27.4759), 10e6, 5e5, 1.6, 0.1},
    {_V(-15.0517, -4.9817, -27.4003), 10e6, 5e5, 1.6, 0.1},
    {_V(15.0901, -4.9817, -27.4759), 10e6, 5e5, 1.6, 0.1},

    {_V(-0.0412, -2.6323, -2.1116), 1e7, 1e5, 3.0},
    {_V(-4.0892, -1.7353, -24.3643), 1e7, 1e5, 3.0},
    {_V(4.0243, -1.7353, -24.3847), 1e7, 1e5},
    {_V(-15.8078, 0.2901, -27.8032), 1e7, 1e5, 3.0},
    {_V(15.6921, 0.2901, -27.8822), 1e7, 1e5, 3.0},
    {_V(4.4373, 5.0727, -26.0741), 1e7, 1e5, 3.0},
    {_V(-4.5627, 5.0727, -26.0515), 1e7, 1e5, 3.0},
    {_V(-4.4162, 8.3727, -20.6627), 1e7, 1e5, 3.0},
    {_V(0.0033, 2.3737, 15.4707), 1e7, 1e5, 3.0},
    {_V(0.0001, 0.0308, 29.9769), 1e7, 1e5, 3.0},
};

//For gear up
static const int ntdvtx_gearup = 10;
static TOUCHDOWNVTX tdvtx_gearup[ntdvtx_gearup] = {
    {_V(-0.0412, -2.6323, -2.1116), 1e7, 1e5, 3.0},
    {_V(-4.0892, -1.7353, -24.3643), 1e7, 1e5, 3.0},
    {_V(4.0243, -1.7353, -24.3847), 1e7, 1e5},
    {_V(-15.8078, 0.2901, -27.8032), 1e7, 1e5, 3.0},
    {_V(15.6921, 0.2901, -27.8822), 1e7, 1e5, 3.0},
    {_V(4.4373, 5.0727, -26.0741), 1e7, 1e5, 3.0},
    {_V(-4.5627, 5.0727, -26.0515), 1e7, 1e5, 3.0},
    {_V(-4.4162, 8.3727, -20.6627), 1e7, 1e5, 3.0},
    {_V(0.0033, 2.3737, 15.4707), 1e7, 1e5, 3.0},
    {_V(0.0001, 0.0308, 29.9769), 1e7, 1e5, 3.0},
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
        int clbkConsumeBufferedKey(DWORD, bool, char *)override;


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
        
        double landing_gear_proc;
        double door_proc;

        AIRFOILHANDLE hwing;
        CTRLSURFHANDLE hlaileron, hraileron, canards;

};

#endif //!__XB70VALKYRIE_H
