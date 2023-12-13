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

// 1. vertical lift component

void VLiftCoeff (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;
	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -15*RAD, 0*RAD,15*RAD,30*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {   0,    -0.56,   -0.56,   -0.16,  0.15,  0.46,  0.56,  0.56,  0.00};
	static const double CM[nabsc]  = {    0,    0.00,   0.00,     0.00,  0.00,  0.00,  0.00,  0.00,  0.00};


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
	*cd = pd + oapiGetInducedDrag (*cl, XB70_VLIFT_A, 0.7) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

// 2. horizontal lift component (vertical stabilisers and body)

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
	*cd = 0.015 + oapiGetInducedDrag (*cl, XB70_HLIFT_A, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}


//Constructor
XB70::XB70(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){
    
    valky_mesh = oapiLoadMesh("XB-70_Valkyrie");

    landing_gear_proc = 0.0;

    door_proc = 0.0;

    landing_gear_status = GEAR_DOWN;

    door_status = DOOR_CLOSED;

    DefineAnimations();

}

//Destructor
XB70::~XB70(){
    
    oapiDeleteMesh(valky_mesh);

    this->VESSEL4::~VESSEL4();
}

void XB70::DefineAnimations(void){

    //Left landing gear
    
    static unsigned int FrontLandingGearGrp[3] = {Front_landing_gear_Id, Front_landing_gear_door1_Id, Front_wheels_landing_gear_Id};
    static MGROUP_ROTATE FrontLandingGear_Rotate(
        0,
        FrontLandingGearGrp,
        3,
        (Front_landing_gear_rotation_Location),
        _V(1, 0, 0),
        (float)(95*RAD)
    );

    static unsigned int FrontLandingGearDoorGrp[1] = {Front_landing_gear_door2_Id};
    static MGROUP_ROTATE FrontLandingGearDoor(
        0,
        FrontLandingGearDoorGrp,
        1,
        (Front_landing_gear_second_door_Location),
        _V(0, 0, 1),
        (float)(100*RAD)
    );

    anim_landing_gear = CreateAnimation(0.0);

    AddAnimationComponent(anim_landing_gear, 0, 0.5, &FrontLandingGear_Rotate);
    AddAnimationComponent(anim_landing_gear, 0.4, 1, &FrontLandingGearDoor);
    
    static unsigned int RearLeftWheels[1] = {Rear_wheels_rear_left_landing_gear_Id};
    static MGROUP_ROTATE RearLeftWheelsRotate(
        0,
        RearLeftWheels,
        1,
        (Axis_left_landing_gear2_Location),
        _V(0, 1, 0),
        (float)(-90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0, 0.3, &RearLeftWheelsRotate);

    static MGROUP_ROTATE RearLeftWheelsRotate2(
        0,
        RearLeftWheels,
        1,
        (Axis_left_landing_gear2_Location),
        _V(0, 0, 1),
        (float)(90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.3, 0.6, &RearLeftWheelsRotate2);

    static unsigned int StowLandingLeftGearGrp[2] = {Rear_left_landing_gear_Id, Rear_wheels_rear_left_landing_gear_Id};
    static MGROUP_ROTATE StowLandingLeftGear(
        0,
        StowLandingLeftGearGrp,
        2,
        (Axis_left_landing_gear_Location),
        _V(1, 0, 0),
        (float)(90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.6, 0.9, &StowLandingLeftGear);

    static unsigned int RearLandingGearDoorsGrp[1] = {Rear_landing_gear_doors_Id};
    static MGROUP_ROTATE RearLandingGearDoors(
        0,
        RearLandingGearDoorsGrp,
        1,
        (Rear_landing_gear_rotation_doors1_Location),
        _V(1, 0, 0),
        (float)(90*RAD)
    );

    AddAnimationComponent(anim_landing_gear,
    0.9, 1, &RearLandingGearDoors);

    static unsigned int RearLeftLandingGearMainDoorGrp[1] = {Rear_landing_gear_left_door_Id};
    static MGROUP_ROTATE RearLeftLandingGearMainDoor(
        0,
        RearLeftLandingGearMainDoorGrp,
        1,
        (Rear_landing_gear_doors1_Location),
        _V(0, 0, 1),
        (float)(-110*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.9, 1, &RearLeftLandingGearMainDoor);

    //Right landing gear

    static unsigned int RearRightWheels[1] = {Rear_wheels_rear_right_landing_gear_Id};
    static MGROUP_ROTATE RearRightWheelsRotate(
        0,
        RearRightWheels,
        1,
        (Axis_right_landing_gear2_Location),
        _V(0, 1, 0),
        (float)(90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0, 0.3, &RearRightWheelsRotate);

    static MGROUP_ROTATE RearRightWheelsRotate2(
        0,
        RearRightWheels,
        1,
        (Axis_right_landing_gear2_Location),
        _V(0, 0, 1),
        (float)(-90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.3, 0.6, &RearRightWheelsRotate2);

    static unsigned int StowLandingRightGearGrp[2] = {Rear_right_landing_gear_Id, Rear_wheels_rear_right_landing_gear_Id};
    static MGROUP_ROTATE StowLandingRightGear(
        0,
        StowLandingRightGearGrp,
        2,
        (Axis_right_landing_gear_Location),
        _V(1, 0, 0),
        (float)(90*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.6, 0.9, &StowLandingRightGear);

    static unsigned int RearRightLandingGearMainDoorGrp[1] = {Rear_landing_gear_right_door_Id};
    static MGROUP_ROTATE RearRightLandingGearMainDoor(
        0,
        RearRightLandingGearMainDoorGrp,
        1,
        (Rear_landing_gear_doors2_Location),
        _V(0, 0, 1),
        (float)(110*RAD)
    );

    AddAnimationComponent(anim_landing_gear, 0.9, 1, &RearRightLandingGearMainDoor);

    //Open/close door

    static unsigned int DoorOpenGrp[1] = {Door_Id};
    static MGROUP_ROTATE DoorOpen(
        0,
        DoorOpenGrp,
        1,
        _V(-1.3062, 0.8235, 19.5891),
        _V(0, 1, 0),
        (float)(2.9670)
    );

    anim_door = CreateAnimation(0.0);

    AddAnimationComponent(anim_door, 0, 1, &DoorOpen);


    ///////Nose cone

    static unsigned int NoseConeGrp[1] = {Nose_cone_Id};
    static MGROUP_ROTATE NoseCone(
        0,
        NoseConeGrp,
        1,
        (Axis_nosecone_Location),
        _V(1, 0, 0),
        (float)(10*RAD)
    );

    anim_nosecone = CreateAnimation(0.0);
    AddAnimationComponent(anim_nosecone, 0, 1, &NoseCone);


    //////Control surfaces   

    static unsigned int ElevatorTrimGrp[2] = {hlaileron_Id, hraileron_Id};
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

    static unsigned int LeftAileronGrp[1] = {hlaileron_Id};
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

    static unsigned int RightAileronGrp[1] = {hraileron_Id};
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

    static unsigned int ElevatorGrp[2] = {hlaileron_Id, hraileron_Id};
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

    static unsigned int CanardsGrp[1] = {Canards_Id};
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

    static unsigned int LRudderGrp[1] = {LRudder_Id};
    static MGROUP_ROTATE LRudder(
        0,
        LRudderGrp,
        1,
        (LRudder_axis_Location),
        _V(0, 1, 0),
        (float)(0.2094)
    );
    anim_lrudder = CreateAnimation(0.5);
    AddAnimationComponent(anim_lrudder, 0, 1, &LRudder);

    static unsigned int RRudderGrp[1] = {RRudder_Id};
    static MGROUP_ROTATE RRudder(
        0,
        RRudderGrp,
        1,
        (RRudder_axis_Location),
        _V(0, 1, 0),
        (float)(0.2094)
    );
    anim_rrudder = CreateAnimation(0.5);
    AddAnimationComponent(anim_rrudder, 0, 1, &RRudder);
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
    SetPMI(XB70_PMI);
    SetMaxWheelbrakeForce(25e5);
    ShiftCentreOfMass(_V(0, -0.8294, 0));
    ShiftCG(_V(0, -0.8294, 0));
    SetWingEffectiveness(2.5);
    SetRotDrag(_V(5, 5, 2.5));
    SetNosewheelSteering(true);

    //Propellant resources
    PROPELLANT_HANDLE JP6 = CreatePropellantResource(XB70_FUELMASS);

    //Define main engine
    th_main[0] = CreateThruster(_V(-3.6722, -0.8294, -25.5365), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    th_main[1] = CreateThruster(_V(-2.2222, -0.8294, -25.5401), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    th_main[2] = CreateThruster(_V(-0.8022, -0.8294, -25.5437), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    th_main[3] = CreateThruster(_V(0.6378, -0.8294, -25.5473), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    th_main[4] = CreateThruster(_V(2.0678, -0.8294, -25.5509), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    th_main[5] = CreateThruster(_V(3.5278, -0.8294, -25.5545), _V(0, 0, 1), XB70_AFTERBRNTH, JP6, XB70_ISP);
    thg_main = CreateThrusterGroup(th_main, 6, THGROUP_MAIN);

    SURFHANDLE exhaust_tex = oapiRegisterExhaustTexture("Exhaust");

    AddExhaust(th_main[0], 15, 1, (ENG0_Location), _V(0, 0, -1));
    AddExhaust(th_main[1], 15, 1, (ENG1_Location), _V(0, 0, -1));
    AddExhaust(th_main[2], 15, 1, (ENG2_Location), _V(0, 0, -1));
    AddExhaust(th_main[3], 15, 1, (ENG3_Location), _V(0, 0, -1));
    AddExhaust(th_main[4], 15, 1, (ENG4_Location), _V(0, 0, -1));
    AddExhaust(th_main[5], 15, 1, (ENG5_Location), _V(0, 0, -1));


    //Add a mesh for the visual
    AddMesh(valky_mesh);

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

    //Contrail effect on canards
    static PARTICLESTREAMSPEC canard_contrails = {
        0, 0.5, .95, 120, 0.03, 10.0, 5, 3.0, 
        PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PLIN, -1.0, 25.0,
		PARTICLESTREAMSPEC::ATM_PLIN, 
    };
    AddParticleStream(&canard_contrails, (Left_canard_contrail_Location), dir, &lvlcontrailcanards);
    AddParticleStream(&canard_contrails, (Right_canard_contrail_Location), dir, &lvlcontrailcanards);


    hwing = CreateAirfoil3(LIFT_VERTICAL, _V(0, -0.8294, 0), VLiftCoeff, 0, XB70_VLIFT_C, (XB70_VLIFT_S*2), XB70_VLIFT_A);

	CreateAirfoil3 (LIFT_HORIZONTAL, (Vertical_tails_Location), HLiftCoeff, 0, XB70_HLIFT_C, (XB70_HLIFT_S*2), XB70_HLIFT_A);
	// vertical stabiliser and body lift and drag components
    
    
	hlaileron = CreateControlSurface3 (AIRCTRL_AILERON, (18.37/2), 1.7, _V(-7.6463, 0.3196, -26.8960), AIRCTRL_AXIS_AUTO, 1.0, anim_raileron);
	hraileron = CreateControlSurface3 (AIRCTRL_AILERON, (18.37/2), 1.7, _V(7.4547, 0.3174, -26.8053), AIRCTRL_AXIS_AUTO, 1.0, anim_laileron);

    canards = CreateControlSurface3(AIRCTRL_ELEVATOR, 38.61, 1.7, _V(-0.0440, 1.4532, 14.7854),AIRCTRL_AXIS_AUTO, 1.0, anim_canards);

    CreateControlSurface3 (AIRCTRL_ELEVATOR, 36.74, 1.7, _V(-0.0833, 0.3068, -26.6097), AIRCTRL_AXIS_AUTO, 1.0, anim_elevator);
	CreateControlSurface3 
    (AIRCTRL_ELEVATORTRIM, 36.74, 1.7, _V(-0.0833, 0.3068, -26.6097), AIRCTRL_AXIS_AUTO, 1.0, anim_elevatortrim);

    CreateControlSurface3(AIRCTRL_RUDDER, 17.76, 1.7, (LRudder_Location), AIRCTRL_AXIS_AUTO, 1.0, 
    anim_lrudder);
    CreateControlSurface3(AIRCTRL_RUDDER, 17.76, 1.7, (RRudder_Location), AIRCTRL_AXIS_AUTO, 1.0, anim_rrudder);

}


//Load landing gear status from scenario file
void XB70::clbkLoadStateEx(FILEHANDLE scn, void *vs){
    
    char *line;

    while(oapiReadScenario_nextline(scn, line)){
        if(!strncasecmp(line, "GEAR", 4)){
            sscanf(line+4, "%d%lf", (int *)&landing_gear_status, &landing_gear_proc);
            SetAnimation(anim_landing_gear, landing_gear_proc);
        } else if (!strncasecmp(line, "DOOR", 4)){
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

void XB70::DeployNoseCone(void){
    ActivateNoseCone((nosecone_status == NOSEC_DEPLOYED || nosecone_status == NOSEC_DEPLOYING) ?
    NOSEC_STOWING : NOSEC_DEPLOYING);
}

void XB70::ActivateNoseCone(NoseConeStatus action){
    nosecone_status = action;
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
    UpdateNoseConeAnimation(simdt);
    UpdateDoorAnimation(simdt);
    lvl = UpdateLvlSndBarrier();
    lvlcontrailcanards = UpdateLvlCanardsEffect();
}

//////////////////////////Functions for gear, door, and Mach 1 contrail effect.

void XB70::UpdateLandingGearAnimation(double simdt) {
    if (landing_gear_status >= GEAR_DEPLOYING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (landing_gear_status == GEAR_DEPLOYING) {
            if (landing_gear_proc > 0.0) landing_gear_proc = std::max(0.0, landing_gear_proc - da);
            else landing_gear_status = GEAR_DOWN;
            SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
        } else {
            if (landing_gear_proc < 1.0) landing_gear_proc = std::min(1.0, landing_gear_proc + da);
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
            if (door_proc > 0.0) door_proc = std::max(0.0, door_proc - da);
            else door_status = DOOR_CLOSED;
        } else {
            if (door_proc < 1.0) door_proc = std::min(1.0, door_proc + da);
            else door_status = DOOR_OPEN;
        }
        SetAnimation(anim_door,door_proc);
    }
}

void XB70::UpdateNoseConeAnimation(double simdt){

    if (nosecone_status >= NOSEC_DEPLOYING) {
            double da = simdt * LANDING_GEAR_OPERATING_SPEED;
            if (nosecone_status == NOSEC_DEPLOYING) {
                if (nosecone_proc > 0.0) nosecone_proc = std::max(0.0, nosecone_proc - da);
                else nosecone_status = NOSEC_DEPLOYED;
            } else {
                if (nosecone_proc < 1.0) nosecone_proc = std::min(1.0, nosecone_proc + da);
                else nosecone_status = NOSEC_STOWED;
            }
            SetAnimation(anim_nosecone, nosecone_proc);
        }
}

double XB70::UpdateLvlSndBarrier(){
    
    double machnumber = GetMachNumber();

    if((machnumber >= 0.999) && (machnumber <= 1.001)){
        return 1.0;
    } else {
        return 0.0;
    }
}

double XB70::UpdateLvlCanardsEffect(){

    double machnumber = GetMachNumber();
    double altitude = GetAltitude();

    if((machnumber > 2) && (altitude > 15000)){
        return 1.0;
    } else {
        return 0.0;
    }

}

int XB70::clbkConsumeBufferedKey(int key, bool down, char *kstate){

    if(key == OAPI_KEY_G && down){
        SetGearDown();
        return 1;
    }
    if(key == OAPI_KEY_K && down){
        CloseDoor();
        return 1;
    }
    if(key == OAPI_KEY_C && down){
        DeployNoseCone();
        return 1;
    }
    return 0;
}


DLLCLBK void InitModule(MODULEHANDLE hModule){

}

DLLCLBK void ExitModule(MODULEHANDLE *hModule){

}



///////////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    
    return new XB70(hvessel, flightmodel);

}

/////////////Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    
    if(vessel) delete(XB70*)vessel;

}