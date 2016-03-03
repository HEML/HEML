/* ------------------------------------------------------------------------*
*                                    HEML                                    *
* -------------------------------------------------------------------------*
* OpenSource code of HEML (Hyper-Elastic Mass Links) method,				 *
* developped at MINES ParisTech, CAOR-Robotics Lab, 2006-2016				 *
* Contact: françois.goulette@mines-paristech.fr								 *
*                                                                            *
* This program is free software: you can redistribute it and/or modify       *
* it under the terms of the GNU General Public License as published by       *
* the Free Software Foundation, either version 3 of the License, or          *
* (at your option) any later version.                                        *             
*                                                                            *
* This program is distributed in the hope that it will be useful,            *
* but WITHOUT ANY WARRANTY; without even the implied warranty of             *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
* GNU General Public License for more details.                               *
*                                                                            *
* You should have received a copy of the GNU General Public License          *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.      *
* -------------------------------------------------------------------------*/

#ifndef MAILLE_H
#define MAILLE_H

#define OK 1
#define ERROR 0

#define MS3D 1
#define STVK 2
#define NH 3
#define Mooney 4

#define NORM2(X) X[0]*X[0]+X[1]*X[1]+X[2]*X[2]

#include <afxwin.h>         // MFC core and standard components
#include <iostream>
//#include <GL/glut.h>
//#include <gl/gl.h>
#include <time.h>

#include <GL/glu.h>
#include <gl/gl.h>
//#include <windows.h>
#include <direct.h>
#include <math.h>

//#include "gstPHANToM.h"
//#include <stdio.h>
//#include <windows.h>
//#include <stdlib.h>
//#include "stdAfx.h"

//#include "PreciseTimer.h"

//PreciseTimer

#ifndef _PRECISETIMER_H_
#define _PRECISE_H_
#include <windows.h>

//More precise Timer for measuring time intervals in microseconds.
//The performance of this Timer is dependent on the performance of the system.
class CPreciseTimer
{
public:
	//CONSTRUCTOR
	CPreciseTimer();

	bool SupportsHighResCounter();
	void StartTimer();
	void StopTimer();
	__int64 GetTime();

private:
	//Auxiliary Function
	void UpdateElapsed();

	//Member variables
	bool m_bRunning;	
	__int64 m_i64Start;
	__int64 m_i64Elapsed;

	//Some auxiliary variables
	__int64 m_i64Counts;
	LARGE_INTEGER m_liCount;

	//Static Variables
	static bool sm_bInit;
	static bool sm_bPerformanceCounter;
	static __int64 sm_i64Freq;
};

inline bool CPreciseTimer::SupportsHighResCounter()
{
	return sm_bPerformanceCounter;
}

//Auxiliary Function
inline void CPreciseTimer::UpdateElapsed()
{
	if(true == sm_bPerformanceCounter)
	{
		QueryPerformanceCounter(&m_liCount);
		m_i64Counts = ((__int64)m_liCount.HighPart << 32) + (__int64)m_liCount.LowPart;
		//Transform in microseconds
		(m_i64Counts *= 1000000) /= sm_i64Freq;
	}
	else
		//Transform milliseconds to microseconds
		m_i64Counts = (__int64)GetTickCount() * 1000;
	if(m_i64Counts > m_i64Start)
		m_i64Elapsed = m_i64Counts - m_i64Start;
	else
		//Eliminate possible number overflow (0x7fffffffffffffff is the maximal __int64 positive number)
		m_i64Elapsed = (0x7fffffffffffffff - m_i64Start) + m_i64Counts;
}
#endif

////////////////////////////////////////////////

	typedef  int * TabIntPtr ;
	typedef  double * TabDblPtr ;

//Class Liaision 
//une liaison est définie par les deux extremités et leurs indices

class Liaison
{
public:
	double *Ext0,*Ext1;
	//double *VExt0,*VExt1
	int indiceExt0,indiceExt1;
	double Long0;
	double Long;
	double Long0X;
	double Long0Y;
	double Long0Z;
	double LongX;
	double LongY;
	double LongZ;
	double ForceX;
	double ForceY;
	double ForceZ;

	double Norm_LongX;
	double Norm_LongY;
	double Norm_LongZ;
	double deltaLong;
	double raideur;

public:
	Liaison(){}
	Liaison(double *ExtA,double *ExtB){
		Ext0=ExtA;
		Ext1=ExtB;
		//Long0=Calc_Long();
		deltaLong=0.;
	}
	double setExt(double *ExtA,int indiceExtA,double *ExtB,int indiceExtB){
		Ext0=ExtA;
		Ext1=ExtB;
		indiceExt0=indiceExtA;
		indiceExt1=indiceExtB;
		Long0X=Ext0[0]-Ext1[0];
		Long0Y=Ext0[1]-Ext1[1];
		Long0Z=Ext0[2]-Ext1[2];
		deltaLong=0.;
		return(Long0=sqrt(Long0X*Long0X+Long0Y*Long0Y+Long0Z*Long0Z));
	}
	double Calc_ALLong()
	{
		LongX=Ext0[0]-Ext1[0];
		LongY=Ext0[1]-Ext1[1];
		LongZ=Ext0[2]-Ext1[2];
		Long=sqrt(LongX*LongX+LongY*LongY+LongZ*LongZ);
		Norm_LongX=LongX/Long;
		Norm_LongY=LongY/Long;
		Norm_LongZ=LongZ/Long;
		return(deltaLong=Long-Long0);
	}
	void Calc_Force()
	{
		Calc_ALLong();
		ForceX =raideur * deltaLong * Norm_LongX;
		ForceY =raideur * deltaLong * Norm_LongY;
		ForceZ =raideur* deltaLong * Norm_LongZ;
	}

	void setRaideur(double Volume_raideur, double moduleYoung)
	{
		raideur=moduleYoung*Volume_raideur/(Long0X*Long0X+Long0Y*Long0Y+Long0Z*Long0Z);
	}
};

class vertice
{
public:
	double *coord;

	//double force[3];
	int numLiaisonAss; // Nombre de liaisons incidentes (associées) à un sommet
	//int *LiaisonAss;
public:
	vertice(){
	}
	void setCoord(double *Coord,int numLiaisonA){
		coord=Coord;
		numLiaisonAss=numLiaisonA;
	}
};


//void __cdecl Col_Detect(void*);

class Maille 
{
public:
	
	// mutex Lock use to protect shared data
	HANDLE hLock;

	//for collision Thread
	HANDLE hCollisionDetect;

	double **sommet;
	double **tab_acc;

	double **d_sommet1;
	double **d_sommet2;
	double **d_sommet3;
	double **d_sommet4;

	vertice *SOMmets;
	double **vitesse;
	double **d_vitesse1;
	double **d_vitesse2;
	double **d_vitesse3;
	double **d_vitesse4;


	int **LinkExt;
	double **LinkVect;
	double **LinkNormVect;
	double *Raideur;
	double *LinkInit;
	double *LinkInit2;
	double *LinkLength;
	double *LinkLength2;

	int *TnumLiaisonAss;
	int **LiaisonAss;
	int Lext0,Lext1;
	int fix;
	
	double LongX;
	double LongY;
	double LongZ;
	double Long;
	double *deltaLong;
	double *deltaLong2;
	double Norm_LongX;
	double Norm_LongY;
	double Norm_LongZ;
	double ForceX;
	double ForceY;
	double ForceZ;
	
	CPreciseTimer oPreciseTimer;
	__int64 i64PreciseTimer ;

	double **force;
	double **forcevisco;
	double **gama;  

	double **acceleration_temp;
//	double vitesse[nbr_points][3];
	int numVertices;
	int numTetra;
	int numtetSurface;
	int numTri;
	int numLiaison;
	int **tetra;

	// Material type
	int MATERIAL_TYPE; 
	// Saint Venant-Kirchhoff
	double **FM1;
	double **FM2;
	double **FM3;
	double Lambda,Mu,E,Nu;
	// Néo-Hookéen
	double **FT1;
	double **FT2;
	double **FT3;
	double lambda0,mu0;
	//Mooney-Rivlin
	double c01;
	double c10;
	double **FMR1;
	double **FMR2;
	double **FMR3;
	double **FR1;
	double **FR2;
	double **FR3;

	int **tetraLink;

	double *InitVolume;
	double *cteVolume;
	double VolumeGlobal;

	int **VoisinTetra;
	int **SurfaceTetra;
	int **SurfaceTri;
	Liaison *RLiaison;
	int num_phantom;

	int numSP;
	int *SurfacePoint;

	double **Collision_Deplacement;
	bool *isCollision;

	double *ForceResultante;

	bool IsMaillePhantom;

	double xTranslate;
	double yTranslate;
	double zTranslate;
	double scale;
	double xRotate;
	double yRotate;
	double zRotate;

	double xSphere;
	double ySphere;
	double zSphere;

	bool construite;
	CString FilePath;

	//Collision Sphere
	double rayon;

	int Col_Tri[30];
	double Col_Point[30][3];
	int nbCol_Tri;
	double Col_Point_Proj[30][3];
	double Col_Center[30][3];
	int nbCol_Center;

	FILE *timeFile;
	CString timeFileName;

	double raideur;
	double raideurVolume;
	double amortissement;
	double *masse;

	double masseVolumique;
	double dt;
	double *gravite; // Vecteur gravité
	double young;
	double yplan;

	double Curs_Pos[3];
	double Curs_Velocity[3];

	clock_t globalTime;
	double temps_calcul;
	double temps_dessin;

	/* valeur tempo pour l'indentation ***/
	double tempo_Ind[201][5];
	int PointAffiche;
	double yIndent_sup;
	double yIndent_inf;
	double force_indent;
	bool StartIndent;
	bool StopIndent;
	bool Indent;

	double ztire;
	bool StartSimu;
	bool StopSimu;
	bool Simu;

	bool enregistre;

	bool booltest;
	bool endCalThread;
	bool endColThread;
	CString mDuration;
	HANDLE		m_hThreadCol;		// Thread Collision handle
	HANDLE		m_hThreadCal;		// Thread Calcul handle

	FILE *pfile_acq;
	FILE *pfile_force;
	bool firstCapture;
	bool Capture;
	bool finCapture;
	int sommetCollision;

public:

	Maille();
//	Maille(gstPHANToM *myPhantom);
	~Maille();
	void dessine_maille();
	void extraire_data();
	void extraire_TetraVoisin();
	void extraire_tetsurface();
	void extraire_surface();
	void charger_surface();
	void extraire_liaisons();
	void charger_liaisons();
	void extraire_SomVoisin();
	void charger_vertices();
	void charger_LiaisonAss();
	void Calcul_Volume();
	void charger_Volume();

	void charger_tetraLink();
	void extraire_tetraLink();
	int init_MRE(double Lambda, double Mu, TabDblPtr *FM1, TabDblPtr *FM2, TabDblPtr *FM3);
	int init_TR(double alpha, TabDblPtr *FT1, TabDblPtr *FT2, TabDblPtr *FT3);
	int init_Mooney(double c01, double c10, TabDblPtr *FR1, TabDblPtr *FR2,TabDblPtr *FR3, TabDblPtr *FMR1, TabDblPtr *FMR2,TabDblPtr *FMR3);     

	void collision(double x,double y,double z);
	void old_Tri_collision(double x,double y,double z);

	void  extraire_SP();
	void  charger_SP();

	inline void control_vol();
	int CalculLongLiaisons(int numLiaison, int **LinkExt, double *deltaLong2);
	int CalculLiaisons(int numLiaison, int **LinkExt, TabDblPtr *sommet,
						   double **LinkNormVect, double *deltaLong, double *deltaLong2);
	int CalculForcesElastiques_MS3D(int numVertices, double **sommet, 
							int numLiaison, int **LinkExt,
							int *TnumLiaisonAss, int **LiaisonAss,double **force);
	int CalculForcesHyperElastiques_STVK(int numVertices, TabDblPtr *sommet, 
							int numTetra, int **tetra, double *InitVolume,
							TabDblPtr *FM1, TabDblPtr *FM2, TabDblPtr *FM3, 
							double **force);
	int CalculForcesHyperElastiques_NH(int numVertices, TabDblPtr *sommet, int numTetra, int **tetra, 
											double *InitVolume, double lambda0, double mu0, 
											TabDblPtr *FT1, TabDblPtr *FT2, TabDblPtr *FT3, double **force);
	int CalculForcesHyperElastiques_Mooney(TabDblPtr *sommet, int numTetra, int **tetra, double *InitVolume, 
											double c01, double c10, TabDblPtr *FR1, TabDblPtr *FR2,TabDblPtr *FR3, TabDblPtr *FMR1, TabDblPtr *FMR2,TabDblPtr *FMR3, double **force);

	int CalculAjoutForcesVolume_MS3D(TabDblPtr *sommet, int numTetra, int **tetra, 
							double *InitVolume, double raideurVolume, double **force);
	int CalculAjoutForcesVolume_NH(TabDblPtr *sommet, int numTetra, int **tetra, 
									   double *InitVolume, double lambda0, double mu0, double **force);
	int CalculAjoutGravite(int numVertices, double **force);
	int CalculAjoutForcesVisqueuses(double amortissement,
										int numVertices, TabDblPtr *sommet,  TabDblPtr *vitesse, 
										int numTetra, int **tetra, double *InitVolume, double **force);
	int CalculAjoutForcesVisqueuses_MS3D(double *Raideur,
									int numVertices, double **vitesse, 
									int **LinkExt,int *TnumLiaisonAss, int **LiaisonAss,
									double **force);

	int CalculForces_et_Integration();

	void Calcul_Masse();
	void charger_Masse();

	void Calcul_Raideur();

	void  SetMaillePhantom(void *phantom);

	void drawString(CString string, GLfloat x, GLfloat y);
	
//	DWORD WINAPI  Col_Detect(LPVOID mp_ppp);
//void __cdecl  Col_Detect(void *);
	UINT Col_Detect(LPVOID pParam)	;

	static UINT WINAPI CalThreadProc(LPVOID pParam);

	static UINT WINAPI ColThreadProc(LPVOID pParam);


//	void capture_proc(gstPHANToM *myPhantom);

	void testptr(double *ppp){
		CString mmessage;
	
	mmessage.Format(_T("testptr: %lf   %lf    %lf   \n"), ppp[0],ppp[1],ppp[2]);
	MessageBox(0,mmessage,"mmessage",0);
	};
/*
	void testwait(){
		int n;
		//while(1)
		//scanf("%d",&n);
	};*/
	bool PointDansTetra(int **Tabtetra,int ntetra, int npoint);
	bool PointDansTri(int **Tabtri,int ntri, int npoint);
	bool TriDansTetra(int **Tabtetra,int ntetra,int **Tabtri,int ntri);
	bool TriDansTetra(int **Tabtetra,int ntetra,int triPoint1,int triPoint2,int triPoint3);

	void testColThread(){ 	
		MessageBox(0,"ok","testColThread",0);}

};

#endif // define MAILLE_H