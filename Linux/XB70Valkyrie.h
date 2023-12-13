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

const double XB70_AFTERBRNTH = 30e4;//120e3; //Max afterburner thrust.

const double LANDING_GEAR_OPERATING_SPEED = 0.06;

const VECTOR3 XB70_CS = {181.48, 642.24, 46.93};

const VECTOR3 XB70_PMI = {245.53, 260.68, 46.93};

const double XB70_VLIFT_C = 23.94; //Chord lenght in meters;

const double XB70_VLIFT_S = 585.07; //Wing area in m^2;

const double XB70_VLIFT_A = 1.751; //Wing aspect ratio;

const double XB70_HLIFT_C = 2.79; //Chord lenght in meters;

const double XB70_HLIFT_S = 17.76; //Wing area in m^2;

const double XB70_HLIFT_A = 1; //Wing aspect ratio;

/*
const double wing_area = 585.0;                    //wing area [sq. m]
const double wing_span = 32.0;                    //wing span [m]
const double wing_aspect_ratio = pow(wing_span, 2) / wing_area; //wing aspect ratio
const double mean_chord_length = wing_area / wing_span; //wing mean chord length
const double wing_effectiveness = 0.4;            //wing effectiveness
//const double XB70_HLIFT_S = 21.74;


const double XB70_HLIFT_C = 5.01;


const double XB70_HLIFT_S = 43.48;

const double XB70_HLIFT_A = 1;

*/



//Define touchdown points
//For gear down
static const int ntdvtx_geardown = 12;
static TOUCHDOWNVTX tdvtx_geardown[ntdvtx_geardown] = {
    {(TDP_Front_landing_gear_Location), 5e6, 5e5, 1.6, 0.1},
    {(TDP_Rear_right_landing_gear_Location), 5e6, 5e5, 3.0, 0.2},
    {(TDP_Rear_left_landing_gear_Location), 5e6, 5e5, 3.0, 0.2},
    {(TDP_Front_Location), 5e6, 5e6, 3.0},
    {(TDP_Rear_left_Location), 5e6, 5e6, 3.0},
    {(TDP_Rear_right_Location), 5e6, 5e6, 3.0},
    {(TDP_Left_wing_Location), 5e6, 5e6, 3.0},
    {(TDP_Right_wing_Location), 5e6, 5e6, 3.0},
    {(TDP_Right_aileron_Location), 5e6, 5e6, 3.0},
    {(TDP_Left_aileron_Location), 5e6, 5e6, 3.0},
    {(TDP_Front_fuselage_Location), 5e6, 5e6, 3.0},
    {(TDP_pitot_probe_Location), 5e6, 5e6, 3.0},
};

//For gear up
static const int ntdvtx_gearup = 9;
static TOUCHDOWNVTX tdvtx_gearup[ntdvtx_gearup] = {
    {(TDP_Front_Location), 5e6, 5e6, 3.0},
    {(TDP_Rear_left_Location), 5e6, 5e6, 3.0},
    {(TDP_Rear_right_Location), 5e6, 5e6, 3.0},
    {(TDP_Left_wing_Location), 5e6, 5e6, 3.0},
    {(TDP_Right_wing_Location), 5e6, 5e6, 3.0},
    {(TDP_Right_aileron_Location), 5e6, 5e6, 3.0},
    {(TDP_Left_aileron_Location), 5e6, 5e6, 3.0},
    {(TDP_Front_fuselage_Location), 5e6, 5e6, 3.0},
    {(TDP_pitot_probe_Location), 5e6, 5e6, 3.0},
};

//XB70 class interface

class XB70: public VESSEL4{
    public:
        enum LandingGearStatus{GEAR_DOWN, GEAR_UP, GEAR_DEPLOYING, GEAR_STOWING} landing_gear_status;
        enum DoorStatus{DOOR_CLOSED, DOOR_OPEN, DOOR_CLOSING, DOOR_OPENING} door_status;
        enum NoseConeStatus{NOSEC_DEPLOYED, NOSEC_STOWED, NOSEC_DEPLOYING, NOSEC_STOWING} nosecone_status;

        XB70(OBJHANDLE hVessel, int flightmodel);
        virtual ~XB70();

        void DefineAnimations(void);
        void ActivateLandingGear(LandingGearStatus action);
        void ActivateDoor(DoorStatus actiondoor);
        void SetGearDown(void);
        void CloseDoor(void);
        void ActivateNoseCone(NoseConeStatus action);
        void DeployNoseCone(void);
        void UpdateLandingGearAnimation(double);
        void UpdateDoorAnimation(double);
        void UpdateNoseConeAnimation(double);
        double UpdateLvlSndBarrier();
        double UpdateLvlCanardsEffect();

        void clbkSetClassCaps(FILEHANDLE cfg)override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs)override;
        void clbkSaveState(FILEHANDLE scn)override;
        void clbkPostStep(double, double, double)override;
        int clbkConsumeBufferedKey(int, bool, char *)override;

        double lvl;
        double lvlcontrailcanards;
    
    private:
        unsigned int anim_landing_gear;
        unsigned int anim_door;
        unsigned int anim_nosecone;
        unsigned int anim_elevatortrim;
        unsigned int anim_laileron;
        unsigned int anim_raileron;
        unsigned int anim_elevator;
        unsigned int anim_canards;
        unsigned int anim_lrudder;
        unsigned int anim_rrudder;

        double landing_gear_proc;
        double door_proc;
        double nosecone_proc;

        AIRFOILHANDLE hwing;
        CTRLSURFHANDLE hlaileron, hraileron, canards;
        MESHHANDLE valky_mesh;

};

#endif //!__XB70VALKYRIE_H
