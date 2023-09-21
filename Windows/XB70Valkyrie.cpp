//Copyright (c) Matías Saibene
//Licenced under the MIT Licence

//==========================================
//          ORBITER MODULE: XB70-Valkyrie
//
//XB70Valkyrie.cpp
//Control module for XB-70 Valkyrie vessel class
//
//==========================================



#define ORBITER_MODULE
#include "XB70Valkyrie.h"
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <algorithm>


// 1. vertical lift component (code from DeltaGlider)

void VLiftCoeff (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;
	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {       0,      0,   -0.4,      0,    0.7,     1,   0.8,     0,      0};
	static const double CM[nabsc]  = {       0,      0,  0.014, 0.0039, -0.006,-0.008,-0.010,     0,      0};
	int i;
	for (i = 0; i < nabsc-1 && AOA[i+1] < aoa; i++);
	if (i < nabsc - 1) {
		double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
		*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
		*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	}
	else {
		*cl = CL[nabsc - 1];
		*cm = CM[nabsc - 1];
	}
	double saoa = sin(aoa);
	double pd = 0.015 + 0.4*saoa*saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag (*cl, 1.5, 0.7) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

// 2. horizontal lift component (code from DeltaGlider)

void HLiftCoeff (VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	int i;
	const int nabsc = 8;
	static const double BETA[nabsc] = {-180*RAD,-135*RAD,-90*RAD,-45*RAD,45*RAD,90*RAD,135*RAD,180*RAD};
	static const double CL[nabsc]   = {       0,    +0.3,      0,   -0.3,  +0.3,     0,   -0.3,      0};
	for (i = 0; i < nabsc-1 && BETA[i+1] < beta; i++);
	if (i < nabsc - 1) {
		*cl = CL[i] + (CL[i + 1] - CL[i]) * (beta - BETA[i]) / (BETA[i + 1] - BETA[i]);
	}
	else {
		*cl = CL[nabsc - 1];
	}
	*cm = 0.0;
	*cd = 0.015 + oapiGetInducedDrag (*cl, 1.5, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}


//Constructor
XB70::XB70(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){
    
    landing_gear_proc = 0.0;

    door_proc = 0.0;

    landing_gear_status = GEAR_DOWN;

    door_status = DOOR_CLOSED;

    DefineAnimations();

}

//Destructor
XB70::~XB70(){

}


void XB70::DefineAnimations(void){

    //Landing gear
    static unsigned int FrontLandingGearGrp[3] = {10, 11, 12};
    static MGROUP_ROTATE FrontLandingGear_Rotate(
        0,
        FrontLandingGearGrp,
        3,
        _V(-0.0869, -2.3828, 2.4933),
        _V(1, 0, 0),
        (float)(2.2689)
    );

    static unsigned int RearLandingGearGrp[7] = {13, 14, 19, 20, 21, 22, 23};
    static MGROUP_ROTATE RearLandingGear_Rotate(
        0,
        RearLandingGearGrp,
        7,
        _V(-0.0869, -1.9791, -11.6312),
        _V(1, 0, 0),
        (float)(1.6057)
    );

    static unsigned int DoorOpenGrp[1] = {6};
    static MGROUP_ROTATE DoorOpen(
        0,
        DoorOpenGrp,
        1,
        _V(-1.3062, 0.8235, 19.5891),
        _V(0, 1, 0),
        (float)(2.9670)
    );

    anim_landing_gear = CreateAnimation(0.0);
    anim_door = CreateAnimation(0.0);

    AddAnimationComponent(anim_landing_gear, 0, 1, &FrontLandingGear_Rotate);
    AddAnimationComponent(anim_landing_gear,
    0, 1, &RearLandingGear_Rotate);

    AddAnimationComponent(anim_door, 0, 1, &DoorOpen);

    static unsigned int ElevatorTrimGrp[2] = {29, 30};
    static MGROUP_ROTATE ElevatorTrim(
        0,
        ElevatorTrimGrp,
        2,
        _V(-0.0793, 0.3068, -25.0097),
        _V(1, 0, 0),
        (float)(0.2094)
    );
    anim_elevatortrim = CreateAnimation(0.5);
    AddAnimationComponent(anim_elevatortrim, 0, 1, &ElevatorTrim);

    static unsigned int LeftAileronGrp[1] = {29};
    static MGROUP_ROTATE LAileron(
        0,
        LeftAileronGrp,
        1,
        _V(-8.6631, 0.3068, -25.1881),
        _V(1, 0, 0),
        (float)(-0.4188)
    );
    anim_laileron = CreateAnimation(0.5);
    AddAnimationComponent(anim_laileron, 0, 1, &LAileron);

    static unsigned int RightAileronGrp[1] = {30};
    static MGROUP_ROTATE RAileron(
        0,
        RightAileronGrp,
        1,
        _V(8.5085, 0.3068, -25.2312),
        _V(1, 0, 0),
        (float)(0.4188)
    );
    anim_raileron = CreateAnimation(0.5);
    AddAnimationComponent(anim_raileron, 0, 1, &RAileron);

    static unsigned int ElevatorGrp[2] = {29, 30};
    static MGROUP_ROTATE Elevator(
        0,
        ElevatorGrp,
        2,
        _V(-0.0793, 0.3068, -25.0097),
        _V(1, 0, 0),
        (float)(0.4188)
    );
    anim_elevator = CreateAnimation(0.5);
    AddAnimationComponent(anim_elevator, 0, 1, &Elevator);

    static unsigned int CanardsGrp[1] = {1};
    static MGROUP_ROTATE Canards(
        0,
        CanardsGrp,
        1,
        _V(0.0977, 1.4979, 15.5177),
        _V(1, 0, 0),
        (float)(0.2617)
    );
    anim_canards = CreateAnimation(0.5);
    AddAnimationComponent(anim_canards, 0, 1, &Canards);
}


// Overloaded callback functions
// Set the capabilities of the vessel class
void XB70::clbkSetClassCaps(FILEHANDLE cfg){

    //Define thrusters
    THRUSTER_HANDLE th_main[6];
    THGROUP_HANDLE thg_main;

    //Physical vessel resources
    SetSize(XB70_SIZE);
    SetEmptyMass(XB70_EMPTYMASS);
    SetCrossSections(XB70_CS);
    SetMaxWheelbrakeForce(25e5);

    //Propellant resources
    PROPELLANT_HANDLE JP6 = CreatePropellantResource(XB70_FUELMASS);

    //Define main engine
    th_main[0] = CreateThruster(_V(-3.6722, -0.8294, -25.5365), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    th_main[1] = CreateThruster(_V(-2.2222, -0.8294, -25.5401), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    th_main[2] = CreateThruster(_V(-0.8022, -0.8294, -25.5437), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    th_main[3] = CreateThruster(_V(0.6378, -0.8294, -25.5473), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    th_main[4] = CreateThruster(_V(2.0678, -0.8294, -25.5509), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    th_main[5] = CreateThruster(_V(3.5278, -0.8294, -25.5545), _V(0, 0, 1), XB70_MAXMAINTH, JP6, XB70_ISP);
    thg_main = CreateThrusterGroup(th_main, 6, THGROUP_MAIN);

    SURFHANDLE exhaust_tex = oapiRegisterExhaustTexture("Exhaust");

    AddExhaust(th_main[0], 15, 1, _V(-3.6722, -0.8294, -25.5365), _V(0, 0, -1));
    AddExhaust(th_main[1], 15, 1, _V(-2.2222, -0.8294, -25.5401), _V(0, 0, -1));
    AddExhaust(th_main[2], 15, 1, _V(-0.8022, -0.8294, -25.5437), _V(0, 0, -1));
    AddExhaust(th_main[3], 15, 1, _V(0.6378, -0.8294, -25.5473), _V(0, 0, -1));
    AddExhaust(th_main[4], 15, 1, _V(2.0678, -0.8294, -25.5509), _V(0, 0, -1));
    AddExhaust(th_main[5], 15, 1, _V(3.5278, -0.8294, -25.5545), _V(0, 0, -1));


    //Add a mesh for the visual
    AddMesh("XB-70_Valkyrie");

    //Sound speed barrier visual effect
    static PARTICLESTREAMSPEC soundbarrierpart = {
		0, 30.0, 15, 350, 0.15, 1, 2, 1, 
		PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0, 2,
		PARTICLESTREAMSPEC::ATM_PLOG, 1e-4, 1
	};
	static VECTOR3 pos = {0, 0, 0};
	static VECTOR3 dir = {0, 0, -1};
	AddParticleStream(&soundbarrierpart, pos, dir, &lvl);


    //Code from DeltaGlider

    hwing = CreateAirfoil3 (LIFT_VERTICAL, _V(-0.0958, 0.0574, -11.9142), VLiftCoeff, 0, XB70_VLIFT_C, XB70_VLIFT_S, XB70_VLIFT_A);
	// wing and body lift+drag components

	CreateAirfoil3 (LIFT_HORIZONTAL, _V(0.0789, 1.8259, -24.2352), HLiftCoeff, 0, XB70_HLIFT_C, XB70_HLIFT_S, XB70_HLIFT_A);
	// vertical stabiliser and body lift and drag components
    
    
	hlaileron = CreateControlSurface3 (AIRCTRL_AILERON, 18.37, 1.7, _V(-7.6463, 0.3196, -26.8960), AIRCTRL_AXIS_XPOS, 1.0, anim_raileron);
	hraileron = CreateControlSurface3 (AIRCTRL_AILERON, 18.37, 1.7, _V(7.4547, 0.3174, -26.8053), AIRCTRL_AXIS_XNEG, 1.0, anim_laileron);

    canards = CreateControlSurface3(AIRCTRL_ELEVATOR, 10.16, 1.7, _V(-0.0440, 1.4532, 14.7854),AIRCTRL_AXIS_XPOS, 1.0, anim_canards);

    CreateControlSurface3 (AIRCTRL_ELEVATOR, 36.74, 1.7, _V(-0.0833, 0.3068, -26.6097), AIRCTRL_AXIS_XPOS, 1.0, anim_elevator);
	CreateControlSurface3 
    (AIRCTRL_ELEVATORTRIM, 36.74, 1.7, _V(-0.0833, 0.3068, -26.6097), AIRCTRL_AXIS_XPOS, 1.0, anim_elevatortrim);
}


//Load landing gear status from scenario file
void XB70::clbkLoadStateEx(FILEHANDLE scn, void *vs){
    
    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        if(!_strnicmp(line, "GEAR", 4)){
            sscanf(line+4, "%d%lf", (int *)&landing_gear_status, &landing_gear_proc);
            SetAnimation(anim_landing_gear, landing_gear_proc);
        } else if (!_strnicmp(line, "DOOR", 4)){
            sscanf(line+4, "%d%lf", (int *)&door_status, &door_proc);
            SetAnimation(anim_door, door_proc);
        } else {
            ParseScenarioLineEx(line, vs);
        }
    }
}

void XB70::clbkSaveState(FILEHANDLE scn){

    char cbuf[256];

    SaveDefaultState(scn);
    sprintf(cbuf, "%d %0.4f", landing_gear_status, landing_gear_proc);
    oapiWriteScenario_string(scn, "GEAR", cbuf);
    
    sprintf(cbuf, "%d %0.4f", door_status, door_proc);
    oapiWriteScenario_string(scn, "DOOR", cbuf);
}


/////////////Logic for gear and door

void XB70::SetGearDown(void){
    ActivateLandingGear((landing_gear_status == GEAR_DOWN || landing_gear_status == GEAR_DEPLOYING) ?
        GEAR_STOWING : GEAR_DEPLOYING);
}

void XB70::CloseDoor(void){
    ActivateDoor((door_status == DOOR_CLOSED || door_status == DOOR_CLOSING) ?
        DOOR_OPENING : DOOR_CLOSING);
}

void XB70::ActivateLandingGear(LandingGearStatus action){
    landing_gear_status = action;
}

void XB70::ActivateDoor(DoorStatus actiondoor){
    door_status = actiondoor;
}

///////////Giving life to animations

void XB70::clbkPostStep(double simt, double simdt, double mjd){
    UpdateLandingGearAnimation(simdt);
    UpdateDoorAnimation(simdt);
    lvl = UpdateParticleLvl();
}

//////////////////////////Functions for gear, door, and Mach 1 contrail effect.

void XB70::UpdateLandingGearAnimation(double simdt) {
    if (landing_gear_status >= GEAR_DEPLOYING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (landing_gear_status == GEAR_DEPLOYING) {
            if (landing_gear_proc > 0.0) landing_gear_proc = max(0.0, landing_gear_proc - da);
            else landing_gear_status = GEAR_DOWN;
            SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
        } else {
            if (landing_gear_proc < 1.0) landing_gear_proc = min(1.0, landing_gear_proc + da);
            else landing_gear_status = GEAR_UP;
            SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
        }
        SetAnimation(anim_landing_gear, landing_gear_proc);
    }
}

void XB70::UpdateDoorAnimation(double simdt) {
    if (door_status >= DOOR_CLOSING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (door_status == DOOR_CLOSING) {
            if (door_proc > 0.0) door_proc = max(0.0, door_proc - da);
            else door_status = DOOR_CLOSED;
        } else {
            if (door_proc < 1.0) door_proc = min(1.0, door_proc + da);
            else door_status = DOOR_OPEN;
        }
        SetAnimation(anim_door,door_proc);
    }
}

double XB70::UpdateParticleLvl(){
    
    double machnumber = GetMachNumber();

    if((machnumber >= 0.999) && (machnumber <= 1.001)){
        return 1.0;
    } else {
        return 0.0;
    }
}

int XB70::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate){

    if(key == OAPI_KEY_G && down){
        SetGearDown();
        return 1;
    }
    if(key == OAPI_KEY_K && down){
        CloseDoor();
        return 1;
    }
    return 0;
}


DLLCLBK void InitModule(HINSTANCE hModule){

}

DLLCLBK void ExitModule(HINSTANCE *hModule){

}



///////////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    
    return new XB70(hvessel, flightmodel);

}

/////////////Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    
    if(vessel) delete(XB70*)vessel;

}