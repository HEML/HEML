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


//////////////////////////////////////////////////////////
//This file is dedicated to calculate the elastic force //
//of soft tissue by using the HEML method               //
//////////////////////////////////////////////////////////


#include "HEML.h"
#include <math.h>
#include <process.h>
#include <atlstr.h>

#define ipointInd 112

static double **ppos;
static double un_neuf=1.0/9.0; 

//////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
// Sub for initialisation of the mesh 
//             and
//      recovery of the config
//(time step dt, density, stiffness, damping and the mesh file name)
///////////////////////////////////////////////////////////////////////////////////////////////////
Maille::Maille(void)
{
	enregistre=false;
	force_indent=0;
	PointAffiche=0;
	firstCapture=false;
	Capture=false;
	finCapture=false;

	VolumeGlobal = 0.;

	// Indentation
	Indent=false;
	StartIndent=false;
	StopIndent=false;

	// Variables for recording the simulation
	StartSimu=true;
	Simu=false;
	StopSimu=false;

	endCalThread=false;
	endColThread=false;
	CString filename;
	CString filename1;

	// The material parameters : read in file 'config.txt'
	filename=_T("config.txt");
	FILE *pfile;
	FilePath=_T("");	
	char * buffer=new char[256];
	char * buff=new char[256];
	char * bu=new char[256];
	char * buffe=new char[256];
	char * buf=new char[256];
	if( (pfile  = fopen( filename, "r" )) == NULL )
	{
		MessageBox(0,"Can not open the File: "+ filename,"Error",0);
		// then use the default configuration
		dt = 0.001;
		masseVolumique = 0.003;
		raideur=100.;
		amortissement = 2.0;
		FilePath="examples\\cube\\cube_5_g";
	}
	else
	{		
		fgets(buf,100,pfile);
		fgets(buffe,100,pfile);
		fscanf(pfile, "%s", buffer);
		fscanf(pfile, "%lf", &masseVolumique);

	// Choice of material and its parameters
	// ======================================
	// MATERIAL_TYPE : 1=MS3D (Mass-Spring 3D), 
	//				   2=STVK (Saint Venant-Kirchhoff)
	//				   3=NH (Néo-Hookéen)
	//                 4=Mooney (Mooney-Rivlin)

		fscanf( pfile, "%i", &MATERIAL_TYPE);
		switch(MATERIAL_TYPE){
		case 1: fscanf( pfile, "%lf %lf", &Nu, &E); break;
		case 2: fscanf( pfile, "%lf %lf", &Nu, &E); Lambda = (Nu*E)/((1.-2.*Nu)*(1.+Nu)); Mu = E/(2.*(1.+Nu)); break;
		case 3: fscanf( pfile, "%lf %lf", &mu0, &lambda0); break;
		case 4: fscanf( pfile, "%lf %lf", &c01, &c10); break;}
	//The other parameters:
		fscanf(pfile, "%lf %lf %lf", &amortissement,&young,&raideurVolume);
		fscanf(pfile, "%s", buff);
		fscanf(pfile, "%lf",&yplan);
		fscanf(pfile, "%lf %lf %lf", &Curs_Pos[0],&Curs_Pos[1],&Curs_Pos[2]);
		fscanf(pfile, "%d", &fix);

		fscanf(pfile, "%s", bu);
		fscanf(pfile, "%lf",&dt);

		fclose(pfile);
		FilePath+=buffer;
	}
	
	// by default, it dosen't use PHANToM
	IsMaillePhantom=false;
	
	// radius of the collision sphere
	rayon=0.7;
	
	// Initialisation of gravity vectors
	gravite=new double[3];
	gravite[0]=0.0; gravite[1]=0.;gravite[2]=0.;

	construite=false;
	
	//Contact trianguler numbers
	nbCol_Tri=-1;
	
	scale=1.;
	xTranslate=0.;
	yTranslate=0.;
	zTranslate=0.;
	xRotate+=0.;
	yRotate+=0.;
	zRotate+=0.;
	booltest=false;
	double duration;
	clock_t start, finish;
	start = clock();
	
#if 1

	CString filename_acq;
	filename_acq=FilePath+".init";
	if( (pfile_acq  = fopen( filename_acq, "w" )) == NULL )
		{
            printf( "\n-> ERROR: cannot open file '%s' !\n", filename_acq);
		}
	else
			printf( "\n-> The file '%s' was opened.\n", filename_acq );

	// treatments
	extraire_data();
	extraire_TetraVoisin();    
	extraire_tetsurface();     
	extraire_surface();        
	charger_surface();

	Calcul_Volume();         
	extraire_liaisons();     
	charger_liaisons();        
	extraire_SomVoisin();    
	charger_vertices();      
	charger_LiaisonAss();      
	extraire_SP();          
	Calcul_Masse();         
	charger_Masse();
	charger_SP();
	charger_Volume();
	extraire_tetraLink();
	charger_tetraLink();

	// Initialisation for Mass-Spring or HEML 
	switch(MATERIAL_TYPE)
    { 
	case MS3D:	// Mass-Spring 3D
		break;
	case STVK: // Saint Venant-Kirchhoff
		FM1=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FM1[ll]=new double[24];
		FM2=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FM2[ll]=new double[24];
		FM3=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FM3[ll]=new double[24];
		init_MRE(Lambda,Mu,FM1,FM2,FM3);
		break;
	case NH: // Néo-Hookéen
		FT1=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FT1[ll]=new double[4];
		FT2=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FT2[ll]=new double[4];
		FT3=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FT3[ll]=new double[4];
		init_TR((mu0/2),FT1,FT2,FT3);
		break;
	case Mooney: // Mooney-Rivlin

		FR1=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FR1[ll]=new double[24];
		FR2=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FR2[ll]=new double[24];
		FR3=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FR3[ll]=new double[24];

		FMR1=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FMR1[ll]=new double[4];
		FMR2=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FMR2[ll]=new double[4];
		FMR3=new TabDblPtr[numTetra];
		for(int ll=0;ll<numTetra;ll++)
			FMR3[ll]=new double[4];

		init_Mooney(c01, c10, FR1, FR2, FR3, FMR1, FMR2, FMR3);
		break;
    }

	fclose(pfile_acq);

#endif
#if 0
	extraire_data();
	extraire_TetraVoisin();
	extraire_tetsurface();
	extraire_surface();
	charger_surface();
	Calcul_Volume();
	extraire_liaisons();
	charger_liaisons();
	extraire_SomVoisin(); 
	charger_vertices();
	extraire_SP();
	Calcul_Masse();
	charger_Masse();
	exit(0);
#endif
	
	finish = clock();
	
	duration = (double)(finish - start) / CLOCKS_PER_SEC;	
	
	CString mDuration;
	
	mDuration.Format(_T("Floating point: %lf\n"), duration);
	construite=true;

	ForceResultante=new double[3];
	
	ForceResultante[0]=0.;
	ForceResultante[1]=0.;
	ForceResultante[2]=0.;

	HANDLE hThread;
	UINT uiThreadId = 0;
	hThread = (HANDLE) _beginthreadex(NULL,	// Security attributes
						  0,	// stack
						  CalThreadProc,	// Thread proc
						  this,	// Thread param
						  CREATE_SUSPENDED,	// creation mode
						  &uiThreadId);	// Thread ID
	
	if (NULL != hThread)
	{
		//SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);
		ResumeThread( hThread );
		m_hThreadCal= hThread;
	}	
}

Maille::~Maille(){ 

}

void Maille::extraire_tetraLink()
{
	int sL0, sL1;
	int sT0,sT1,sT2,sT3;
	CString filename;
	filename=FilePath+".tetraLink";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );

	for(int i=0;i<numTetra;i++)
	{
		sT0=tetra[i][0];
		sT1=tetra[i][1];
		sT2=tetra[i][2];
		sT3=tetra[i][3];

		for(int j=0;j<numLiaison;j++)
		{
			sL0=LinkExt[j][0];
			sL1=LinkExt[j][1];
			if( (sL0==sT0)||(sL0==sT1)||(sL0==sT2)||(sL0==sT3) )
			{
				if ((sL1==sT0)||(sL1==sT1)||(sL1==sT2)||(sL1==sT3))
				{
					fprintf(pfile,"%d ", j);
				}
			}
		}
		fprintf(pfile,"\n");
	}
fclose(pfile);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Maille::charger_tetraLink()
{
	int L;
	CString filename;
	filename=FilePath+".tetraLink";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );

	int s0,s1;
	for(int i=0;i<numTetra;i++)
	{	
		for(int j=0;j<6;j++)
		{
			fscanf( pfile, "%d",&L);

			s0=LinkExt[L][0];
			s1=LinkExt[L][1];
			if(	  (	(s0==tetra[i][0])&&(s1==tetra[i][1])  ) ||   (	(s0==tetra[i][1])&&(s1==tetra[i][0])  )   )
				tetraLink[i][0]=L;

			if(	  (	(s0==tetra[i][0])&&(s1==tetra[i][2])  ) ||   (	(s0==tetra[i][2])&&(s1==tetra[i][0])  )   )
				tetraLink[i][1]=L;

			if(	  (	(s0==tetra[i][0])&&(s1==tetra[i][3])  ) ||   (	(s0==tetra[i][3])&&(s1==tetra[i][0])  )   )
				tetraLink[i][2]=L;

			if(	  (	(s0==tetra[i][1])&&(s1==tetra[i][2])  ) ||   (	(s0==tetra[i][2])&&(s1==tetra[i][1])  )   )
				tetraLink[i][3]=L;

			if(	  (	(s0==tetra[i][2])&&(s1==tetra[i][3])  ) ||   (	(s0==tetra[i][3])&&(s1==tetra[i][2])  )   )
				tetraLink[i][4]=L;

			if(	  (	(s0==tetra[i][3])&&(s1==tetra[i][1])  ) ||   (	(s0==tetra[i][1])&&(s1==tetra[i][3])  )   )
				tetraLink[i][5]=L;
		}
	}

fclose(pfile);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int Maille::init_MRE(double Lambda, double Mu, TabDblPtr *FM1, TabDblPtr *FM2, TabDblPtr *FM3)
// Initialisation for HEML Saint Venant Kirchhoff
// In : Lambda, Mu
// Out : FM1, FM2, FM3
{
	double V[9];
	double MW[36];

	double D[54]={
    1.0000 ,   0.5000 ,   0.5000 ,
    0.5000 ,       0. ,       0. ,
    0.5000 ,       0. ,       0. ,

        0. ,   0.5000 ,       0. ,
    0.5000 ,   1.0000 ,   0.5000 ,
        0. ,   0.5000 ,       0. ,

        0. ,       0. ,   0.5000 ,
        0. ,       0. ,   0.5000 ,
    0.5000 ,   0.5000 ,   1.0000 ,

        0. ,  -0.5000 ,       0. ,
   -0.5000 ,       0. ,       0. ,
        0. ,       0. ,       0. ,

        0. ,       0. ,       0. ,
        0. ,       0. ,  -0.5000 ,
        0. ,  -0.5000 ,       0. ,

        0. ,       0. ,  -0.5000 ,
        0. ,       0. ,       0. ,
   -0.5000 ,       0. ,       0. 
	};

	double B[9];
	double DB[9];
	double C[54];
	double Det;
	double InvDet;
	double VT[6];
	double MT[36];
	double DL1[24]={
		-2., 2., 0., 0.,	0., 0., 0., 0.,		0., 0., 0., 0.,		
		0., 2., -2., 0.,		0., 0., 0., 0.,		0., 2., 0., -2.
	};
	double DL2[24]={
		0., 0., 0., 0.,		-2., 0., 2., 0.,	0., 0., 0., 0.,		
			0., -2., 2., 0.,		0., 0., 2., -2.,	0., 0., 0., 0.
	};
	double DL3[24]={
		0., 0., 0., 0.,		0., 0., 0., 0.,		-2., 0., 0., 2.,	
			0., 0., 0., 0.,			0., 0., -2., 2.,	0., -2., 0., 2.
	};

	int i,j,k;
	int m_0, m_1, m_2, m_3;

	CString filename_force;
	filename_force=FilePath+".force";
	if( (pfile_force  = fopen( filename_force, "w" )) == NULL )
		{
            printf( "\n-> ERROR: cannot open file '%s' !\n", filename_force);
		}
	else
			printf( "\n-> The file '%s' was opened.\n", filename_force );

	for(i=0;i<numTetra;i++)
	{
		// Sommets du tétraèdre
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];
		for(j=0;j<3;j++)
		{
			// Vecteurs des arêtes V
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		// B=inv(V)
		Det =V[0]*V[4]*V[8]
			+V[1]*V[5]*V[6]
			+V[2]*V[3]*V[7]
            -V[6]*V[4]*V[2]
			-V[7]*V[5]*V[0]
			-V[8]*V[3]*V[1];

		InvDet=1./Det;
		
		B[0]=  (V[4]*V[8]-V[7]*V[5])*InvDet;
		B[1]= -(V[1]*V[8]-V[7]*V[2])*InvDet;
		B[2]=  (V[1]*V[5]-V[2]*V[4])*InvDet;
							   
		B[3]= -(V[3]*V[8]-V[6]*V[5])*InvDet;
		B[4]=  (V[0]*V[8]-V[6]*V[2])*InvDet;
		B[5]= -(V[0]*V[5]-V[3]*V[2])*InvDet;
							   
		B[6]=  (V[3]*V[7]-V[6]*V[4])*InvDet;
		B[7]= -(V[0]*V[7]-V[6]*V[1])*InvDet;
		B[8]=  (V[0]*V[4]-V[3]*V[1])*InvDet;

		// Matrice C
		for(j=0;j<6;j++)
		{
			// C(:,:,j)=B'*D(:,:,j)*B
			for(k=0;k<3;k++)
			{
				DB[3*k]=		D[9*j+3*k]  *B[0] + D[9*j+3*k+1]*B[3] + D[9*j+3*k+2]*B[6];
				DB[3*k+1]=	    D[9*j+3*k]  *B[1] + D[9*j+3*k+1]*B[4] + D[9*j+3*k+2]*B[7];
				DB[3*k+2]=	    D[9*j+3*k]  *B[2] + D[9*j+3*k+1]*B[5] + D[9*j+3*k+2]*B[8];
			}
			for(k=0;k<3;k++)
			{
				C[9*j+3*k]=			B[k]  *DB[0] + B[k+3]*DB[3] + B[k+6]*DB[6];
				C[9*j+3*k+1]=	    B[k]  *DB[1] + B[k+3]*DB[4] + B[k+6]*DB[7];
				C[9*j+3*k+2]=	    B[k]  *DB[2] + B[k+3]*DB[5] + B[k+6]*DB[8];
			}
		}
		
		// VT et MT
		for(j=0;j<6;j++)
		{
			VT[j]=C[9*j]+ C[9*j+4] +C[9*j+8];
			for(k=0;k<6;k++)
			{
				MT[6*j+k]=  C[9*j+0]*C[9*k+0] + C[9*j+1]*C[9*k+3] +C[9*j+2]*C[9*k+6] +
							C[9*j+3]*C[9*k+1] + C[9*j+4]*C[9*k+4] +C[9*j+5]*C[9*k+7] +
							C[9*j+6]*C[9*k+2] + C[9*j+7]*C[9*k+5] +C[9*j+8]*C[9*k+8] ;
			}
		}

		//MW = (lambda/8)*VT*VT'+(mu/4)*MT;
		for(j=0;j<6;j++)
		{
            MW[6*j]  =	(Lambda/8.)*VT[j]*VT[0] + (Mu/4.)*MT[6*j];
            MW[6*j+1]=	(Lambda/8.)*VT[j]*VT[1] + (Mu/4.)*MT[6*j+1];	
            MW[6*j+2]=	(Lambda/8.)*VT[j]*VT[2] + (Mu/4.)*MT[6*j+2];	
            MW[6*j+3]=	(Lambda/8.)*VT[j]*VT[3] + (Mu/4.)*MT[6*j+3];	
            MW[6*j+4]=	(Lambda/8.)*VT[j]*VT[4] + (Mu/4.)*MT[6*j+4];	
            MW[6*j+5]=	(Lambda/8.)*VT[j]*VT[5] + (Mu/4.)*MT[6*j+5];	
		}

		// FM1, FM2, FM3
		for(j=0;j<6;j++)
		{
			FM1[i][j*4]  =MW[j*6]*DL1[0]+MW[j*6+1]*DL1[4]+MW[j*6+2]*DL1[8]+MW[j*6+3]*DL1[12]+MW[j*6+4]*DL1[16]+MW[j*6+5]*DL1[20];
			FM1[i][j*4+1]=MW[j*6]*DL1[1]+MW[j*6+1]*DL1[5]+MW[j*6+2]*DL1[9]+MW[j*6+3]*DL1[13]+MW[j*6+4]*DL1[17]+MW[j*6+5]*DL1[21];
			FM1[i][j*4+2]=MW[j*6]*DL1[2]+MW[j*6+1]*DL1[6]+MW[j*6+2]*DL1[10]+MW[j*6+3]*DL1[14]+MW[j*6+4]*DL1[18]+MW[j*6+5]*DL1[22];
			FM1[i][j*4+3]=MW[j*6]*DL1[3]+MW[j*6+1]*DL1[7]+MW[j*6+2]*DL1[11]+MW[j*6+3]*DL1[15]+MW[j*6+4]*DL1[19]+MW[j*6+5]*DL1[23];

			FM2[i][j*4]  =MW[j*6]*DL2[0]+MW[j*6+1]*DL2[4]+MW[j*6+2]*DL2[8]+MW[j*6+3]* DL2[12]+MW[j*6+4]*DL2[16]+MW[j*6+5]*DL2[20];
			FM2[i][j*4+1]=MW[j*6]*DL2[1]+MW[j*6+1]*DL2[5]+MW[j*6+2]*DL2[9]+MW[j*6+3]* DL2[13]+MW[j*6+4]*DL2[17]+MW[j*6+5]*DL2[21];
			FM2[i][j*4+2]=MW[j*6]*DL2[2]+MW[j*6+1]*DL2[6]+MW[j*6+2]*DL2[10]+MW[j*6+3]*DL2[14]+MW[j*6+4]*DL2[18]+MW[j*6+5]*DL2[22];
			FM2[i][j*4+3]=MW[j*6]*DL2[3]+MW[j*6+1]*DL2[7]+MW[j*6+2]*DL2[11]+MW[j*6+3]*DL2[15]+MW[j*6+4]*DL2[19]+MW[j*6+5]*DL2[23];
																														 
			FM3[i][j*4]  =MW[j*6]*DL3[0]+MW[j*6+1]*DL3[4]+MW[j*6+2]*DL3[8]+MW[j*6+3]* DL3[12]+MW[j*6+4]*DL3[16]+MW[j*6+5]*DL3[20];
			FM3[i][j*4+1]=MW[j*6]*DL3[1]+MW[j*6+1]*DL3[5]+MW[j*6+2]*DL3[9]+MW[j*6+3]* DL3[13]+MW[j*6+4]*DL3[17]+MW[j*6+5]*DL3[21];
			FM3[i][j*4+2]=MW[j*6]*DL3[2]+MW[j*6+1]*DL3[6]+MW[j*6+2]*DL3[10]+MW[j*6+3]*DL3[14]+MW[j*6+4]*DL3[18]+MW[j*6+5]*DL3[22];
			FM3[i][j*4+3]=MW[j*6]*DL3[3]+MW[j*6+1]*DL3[7]+MW[j*6+2]*DL3[11]+MW[j*6+3]*DL3[15]+MW[j*6+4]*DL3[19]+MW[j*6+5]*DL3[23];
		}
	}
	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int Maille::init_TR(double alpha, TabDblPtr *FT1, TabDblPtr *FT2, TabDblPtr *FT3)
// Initialisation for HEML Néo-Hookéen
// In : alpha
// Out : FT1, FT2, FT3
{
	double V[9];

	double D[54]={
    1.0000 ,   0.5000 ,   0.5000 ,
    0.5000 ,       0. ,       0. ,
    0.5000 ,       0. ,       0. ,

        0. ,   0.5000 ,       0. ,
    0.5000 ,   1.0000 ,   0.5000 ,
        0. ,   0.5000 ,       0. ,

        0. ,       0. ,   0.5000 ,
        0. ,       0. ,   0.5000 ,
    0.5000 ,   0.5000 ,   1.0000 ,

        0. ,  -0.5000 ,       0. ,
   -0.5000 ,       0. ,       0. ,
        0. ,       0. ,       0. ,

        0. ,       0. ,       0. ,
        0. ,       0. ,  -0.5000 ,
        0. ,  -0.5000 ,       0. ,

        0. ,       0. ,  -0.5000 ,
        0. ,       0. ,       0. ,
   -0.5000 ,       0. ,       0. 
	};

	double B[9];
	double DB[9];
	double C[54];
	double Det;
	double InvDet;
	double VT[6];
	double DL1[24]={
		-2., 2., 0., 0.,	0., 0., 0., 0.,		0., 0., 0., 0.,		
		0., 2., -2., 0.,		0., 0., 0., 0.,		0., 2., 0., -2.
	};
	double DL2[24]={
		0., 0., 0., 0.,		-2., 0., 2., 0.,	0., 0., 0., 0.,		
			0., -2., 2., 0.,		0., 0., 2., -2.,	0., 0., 0., 0.
	};
	double DL3[24]={
		0., 0., 0., 0.,		0., 0., 0., 0.,		-2., 0., 0., 2.,	
			0., 0., 0., 0.,			0., 0., -2., 2.,	0., -2., 0., 2.
	};

	int i,j,k;
	int m_0, m_1, m_2, m_3;
	for(i=0;i<numTetra;i++)
	{
		// nodes of tetrahedrons
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];
		for(j=0;j<3;j++)
		{
			// edges vectors V
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		// B=inv(V)
		Det =V[0]*V[4]*V[8]
			+V[1]*V[5]*V[6]
			+V[2]*V[3]*V[7]
            -V[6]*V[4]*V[2]
			-V[7]*V[5]*V[0]
			-V[8]*V[3]*V[1];

		InvDet=1./Det;
		
		B[0]=  (V[4]*V[8]-V[7]*V[5])*InvDet;
		B[1]= -(V[1]*V[8]-V[7]*V[2])*InvDet;
		B[2]=  (V[1]*V[5]-V[2]*V[4])*InvDet;
							   
		B[3]= -(V[3]*V[8]-V[6]*V[5])*InvDet;
		B[4]=  (V[0]*V[8]-V[6]*V[2])*InvDet;
		B[5]= -(V[0]*V[5]-V[3]*V[2])*InvDet;
							   
		B[6]=  (V[3]*V[7]-V[6]*V[4])*InvDet;
		B[7]= -(V[0]*V[7]-V[6]*V[1])*InvDet;
		B[8]=  (V[0]*V[4]-V[3]*V[1])*InvDet;

		// Matrix C
		for(j=0;j<6;j++)
		{
			// C(:,:,j)=B'*D(:,:,j)*B
			for(k=0;k<3;k++)
			{
				DB[3*k]=		D[9*j+3*k]  *B[0] + D[9*j+3*k+1]*B[3] + D[9*j+3*k+2]*B[6];
				DB[3*k+1]=	    D[9*j+3*k]  *B[1] + D[9*j+3*k+1]*B[4] + D[9*j+3*k+2]*B[7];
				DB[3*k+2]=	    D[9*j+3*k]  *B[2] + D[9*j+3*k+1]*B[5] + D[9*j+3*k+2]*B[8];
			}
			for(k=0;k<3;k++)
			{
				C[9*j+3*k]=			B[k]  *DB[0] + B[k+3]*DB[3] + B[k+6]*DB[6];
				C[9*j+3*k+1]=	    B[k]  *DB[1] + B[k+3]*DB[4] + B[k+6]*DB[7];
				C[9*j+3*k+2]=	    B[k]  *DB[2] + B[k+3]*DB[5] + B[k+6]*DB[8];
			}
		}
		
		// VT
		for(j=0;j<6;j++)
			VT[j]=C[9*j]+ C[9*j+4] +C[9*j+8];

		FT1[i][0]=alpha*(VT[0]*DL1[0]+VT[1]*DL1[4]+VT[2]*DL1[8]+VT[3]*DL1[12]+VT[4]*DL1[16]+VT[5]*DL1[20]);
		FT1[i][1]=alpha*(VT[0]*DL1[1]+VT[1]*DL1[5]+VT[2]*DL1[9]+VT[3]*DL1[13]+VT[4]*DL1[17]+VT[5]*DL1[21]);
		FT1[i][2]=alpha*(VT[0]*DL1[2]+VT[1]*DL1[6]+VT[2]*DL1[10]+VT[3]*DL1[14]+VT[4]*DL1[18]+VT[5]*DL1[22]);
		FT1[i][3]=alpha*(VT[0]*DL1[3]+VT[1]*DL1[7]+VT[2]*DL1[11]+VT[3]*DL1[15]+VT[4]*DL1[19]+VT[5]*DL1[23]);

		FT2[i][0]=alpha*(VT[0]*DL2[0]+VT[1]*DL2[4]+VT[2]*DL2[8]+VT[3]*DL2[12]+VT[4]*DL2[16]+VT[5]*DL2[20]);
		FT2[i][1]=alpha*(VT[0]*DL2[1]+VT[1]*DL2[5]+VT[2]*DL2[9]+VT[3]*DL2[13]+VT[4]*DL2[17]+VT[5]*DL2[21]);
		FT2[i][2]=alpha*(VT[0]*DL2[2]+VT[1]*DL2[6]+VT[2]*DL2[10]+VT[3]*DL2[14]+VT[4]*DL2[18]+VT[5]*DL2[22]);
		FT2[i][3]=alpha*(VT[0]*DL2[3]+VT[1]*DL2[7]+VT[2]*DL2[11]+VT[3]*DL2[15]+VT[4]*DL2[19]+VT[5]*DL2[23]);

		FT3[i][0]=alpha*(VT[0]*DL3[0]+VT[1]*DL3[4]+VT[2]*DL3[8]+VT[3]*DL3[12]+VT[4]*DL3[16]+VT[5]*DL3[20]);
		FT3[i][1]=alpha*(VT[0]*DL3[1]+VT[1]*DL3[5]+VT[2]*DL3[9]+VT[3]*DL3[13]+VT[4]*DL3[17]+VT[5]*DL3[21]);
		FT3[i][2]=alpha*(VT[0]*DL3[2]+VT[1]*DL3[6]+VT[2]*DL3[10]+VT[3]*DL3[14]+VT[4]*DL3[18]+VT[5]*DL3[22]);
		FT3[i][3]=alpha*(VT[0]*DL3[3]+VT[1]*DL3[7]+VT[2]*DL3[11]+VT[3]*DL3[15]+VT[4]*DL3[19]+VT[5]*DL3[23]);
	}

	return OK;
}

int Maille::init_Mooney(double c01, double c10, TabDblPtr *FR1, TabDblPtr *FR2,TabDblPtr *FR3, TabDblPtr *FMR1, TabDblPtr *FMR2,TabDblPtr *FMR3)
	// Initialisation for HEML Mooney-Rivlin
	// In : the nodes
	// Out : Vtr, Mtr
{
	double V[9];
	double VtrL;  //Vtr*L

	double L[6];
	double Vtr[6];
	double Mtr[36];
	double MW[36];

	double dwdl[6];
	double LMtr[6]; //Lt*Mtr

	double D[54]={
    1.0000 ,   0.5000 ,   0.5000 ,
    0.5000 ,       0. ,       0. ,
    0.5000 ,       0. ,       0. ,

        0. ,   0.5000 ,       0. ,
    0.5000 ,   1.0000 ,   0.5000 ,
        0. ,   0.5000 ,       0. ,

        0. ,       0. ,   0.5000 ,
        0. ,       0. ,   0.5000 ,
    0.5000 ,   0.5000 ,   1.0000 ,

        0. ,  -0.5000 ,       0. ,
   -0.5000 ,       0. ,       0. ,
        0. ,       0. ,       0. ,

        0. ,       0. ,       0. ,
        0. ,       0. ,  -0.5000 ,
        0. ,  -0.5000 ,       0. ,

        0. ,       0. ,  -0.5000 ,
        0. ,       0. ,       0. ,
   -0.5000 ,       0. ,       0. 
	};

	double B[9];
	double DB[9];
	double C[54];
	double Det;
	double InvDet;
	double DL1[24]={
		-2., 2., 0., 0.,	0., 0., 0., 0.,		0., 0., 0., 0.,		
		0., 2., -2., 0.,		0., 0., 0., 0.,		0., 2., 0., -2.
	};
	double DL2[24]={
		0., 0., 0., 0.,		-2., 0., 2., 0.,	0., 0., 0., 0.,		
			0., -2., 2., 0.,		0., 0., 2., -2.,	0., 0., 0., 0.
	};
	double DL3[24]={
		0., 0., 0., 0.,		0., 0., 0., 0.,		-2., 0., 0., 2.,	
			0., 0., 0., 0.,			0., 0., -2., 2.,	0., -2., 0., 2.
	};

	int i,j,k;
	int m_0, m_1, m_2, m_3;
	for(i=0;i<numTetra;i++)
	{
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];
		for(j=0;j<3;j++)
		{
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		Det =V[0]*V[4]*V[8]
			+V[1]*V[5]*V[6]
			+V[2]*V[3]*V[7]
            -V[6]*V[4]*V[2]
			-V[7]*V[5]*V[0]
			-V[8]*V[3]*V[1];

		InvDet=1./Det;
		
		B[0]=  (V[4]*V[8]-V[7]*V[5])*InvDet;
		B[1]= -(V[1]*V[8]-V[7]*V[2])*InvDet;
		B[2]=  (V[1]*V[5]-V[2]*V[4])*InvDet;
							   
		B[3]= -(V[3]*V[8]-V[6]*V[5])*InvDet;
		B[4]=  (V[0]*V[8]-V[6]*V[2])*InvDet;
		B[5]= -(V[0]*V[5]-V[3]*V[2])*InvDet;
							   
		B[6]=  (V[3]*V[7]-V[6]*V[4])*InvDet;
		B[7]= -(V[0]*V[7]-V[6]*V[1])*InvDet;
		B[8]=  (V[0]*V[4]-V[3]*V[1])*InvDet;

		for(j=0;j<6;j++)
		{
			for(k=0;k<3;k++)
			{
				DB[3*k]=		D[9*j+3*k]  *B[0] + D[9*j+3*k+1]*B[3] + D[9*j+3*k+2]*B[6];
				DB[3*k+1]=	    D[9*j+3*k]  *B[1] + D[9*j+3*k+1]*B[4] + D[9*j+3*k+2]*B[7];
				DB[3*k+2]=	    D[9*j+3*k]  *B[2] + D[9*j+3*k+1]*B[5] + D[9*j+3*k+2]*B[8];
			}
			for(k=0;k<3;k++)
			{
				C[9*j+3*k]=			B[k]  *DB[0] + B[k+3]*DB[3] + B[k+6]*DB[6];
				C[9*j+3*k+1]=	    B[k]  *DB[1] + B[k+3]*DB[4] + B[k+6]*DB[7];
				C[9*j+3*k+2]=	    B[k]  *DB[2] + B[k+3]*DB[5] + B[k+6]*DB[8];
			}
		}
		
		// Vtr et Mtr
		for(j=0;j<6;j++)
		{
			Vtr[j]=C[9*j]+ C[9*j+4] +C[9*j+8];
			for(k=0;k<6;k++)
			{
				Mtr[6*j+k]=  C[9*j+0]*C[9*k+0] + C[9*j+1]*C[9*k+3] +C[9*j+2]*C[9*k+6] +
							C[9*j+3]*C[9*k+1] + C[9*j+4]*C[9*k+4] +C[9*j+5]*C[9*k+7] +
							C[9*j+6]*C[9*k+2] + C[9*j+7]*C[9*k+5] +C[9*j+8]*C[9*k+8] ;
			}

		}
		//MW = 1/2*c10*(Vtr*Vtr'-Mtr);
		for(j=0;j<6;j++)
		{
            MW[6*j]  =	0.5*c10*Vtr[j]*Vtr[0] - 0.5*c10*Mtr[6*j];
            MW[6*j+1]=	0.5*c10*Vtr[j]*Vtr[1] - 0.5*c10*Mtr[6*j+1];	
            MW[6*j+2]=	0.5*c10*Vtr[j]*Vtr[2] - 0.5*c10*Mtr[6*j+2];	
            MW[6*j+3]=	0.5*c10*Vtr[j]*Vtr[3] - 0.5*c10*Mtr[6*j+3];	
            MW[6*j+4]=	0.5*c10*Vtr[j]*Vtr[4] - 0.5*c10*Mtr[6*j+4];	
            MW[6*j+5]=	0.5*c10*Vtr[j]*Vtr[5] - 0.5*c10*Mtr[6*j+5];	
		}
		
		for(j=0;j<6;j++)
		{
			FR1[i][j*4]  =MW[j*6]*DL1[0]+MW[j*6+1]*DL1[4]+MW[j*6+2]*DL1[8]+MW[j*6+3]*DL1[12]+MW[j*6+4]*DL1[16]+MW[j*6+5]*DL1[20];
			FR1[i][j*4+1]=MW[j*6]*DL1[1]+MW[j*6+1]*DL1[5]+MW[j*6+2]*DL1[9]+MW[j*6+3]*DL1[13]+MW[j*6+4]*DL1[17]+MW[j*6+5]*DL1[21];
			FR1[i][j*4+2]=MW[j*6]*DL1[2]+MW[j*6+1]*DL1[6]+MW[j*6+2]*DL1[10]+MW[j*6+3]*DL1[14]+MW[j*6+4]*DL1[18]+MW[j*6+5]*DL1[22];
			FR1[i][j*4+3]=MW[j*6]*DL1[3]+MW[j*6+1]*DL1[7]+MW[j*6+2]*DL1[11]+MW[j*6+3]*DL1[15]+MW[j*6+4]*DL1[19]+MW[j*6+5]*DL1[23];

			FR2[i][j*4]  =MW[j*6]*DL2[0]+MW[j*6+1]*DL2[4]+MW[j*6+2]*DL2[8]+MW[j*6+3]* DL2[12]+MW[j*6+4]*DL2[16]+MW[j*6+5]*DL2[20];
			FR2[i][j*4+1]=MW[j*6]*DL2[1]+MW[j*6+1]*DL2[5]+MW[j*6+2]*DL2[9]+MW[j*6+3]* DL2[13]+MW[j*6+4]*DL2[17]+MW[j*6+5]*DL2[21];
			FR2[i][j*4+2]=MW[j*6]*DL2[2]+MW[j*6+1]*DL2[6]+MW[j*6+2]*DL2[10]+MW[j*6+3]*DL2[14]+MW[j*6+4]*DL2[18]+MW[j*6+5]*DL2[22];
			FR2[i][j*4+3]=MW[j*6]*DL2[3]+MW[j*6+1]*DL2[7]+MW[j*6+2]*DL2[11]+MW[j*6+3]*DL2[15]+MW[j*6+4]*DL2[19]+MW[j*6+5]*DL2[23];
																														 
			FR3[i][j*4]  =MW[j*6]*DL3[0]+MW[j*6+1]*DL3[4]+MW[j*6+2]*DL3[8]+MW[j*6+3]* DL3[12]+MW[j*6+4]*DL3[16]+MW[j*6+5]*DL3[20];
			FR3[i][j*4+1]=MW[j*6]*DL3[1]+MW[j*6+1]*DL3[5]+MW[j*6+2]*DL3[9]+MW[j*6+3]* DL3[13]+MW[j*6+4]*DL3[17]+MW[j*6+5]*DL3[21];
			FR3[i][j*4+2]=MW[j*6]*DL3[2]+MW[j*6+1]*DL3[6]+MW[j*6+2]*DL3[10]+MW[j*6+3]*DL3[14]+MW[j*6+4]*DL3[18]+MW[j*6+5]*DL3[22];
			FR3[i][j*4+3]=MW[j*6]*DL3[3]+MW[j*6+1]*DL3[7]+MW[j*6+2]*DL3[11]+MW[j*6+3]*DL3[15]+MW[j*6+4]*DL3[19]+MW[j*6+5]*DL3[23];
		}

		FMR1[i][0]=c01*(Vtr[0]*DL1[0]+Vtr[1]*DL1[4]+Vtr[2]*DL1[8]+Vtr[3]*DL1[12]+Vtr[4]*DL1[16]+Vtr[5]*DL1[20]);
		FMR1[i][1]=c01*(Vtr[0]*DL1[1]+Vtr[1]*DL1[5]+Vtr[2]*DL1[9]+Vtr[3]*DL1[13]+Vtr[4]*DL1[17]+Vtr[5]*DL1[21]);
		FMR1[i][2]=c01*(Vtr[0]*DL1[2]+Vtr[1]*DL1[6]+Vtr[2]*DL1[10]+Vtr[3]*DL1[14]+Vtr[4]*DL1[18]+Vtr[5]*DL1[22]);
		FMR1[i][3]=c01*(Vtr[0]*DL1[3]+Vtr[1]*DL1[7]+Vtr[2]*DL1[11]+Vtr[3]*DL1[15]+Vtr[4]*DL1[19]+Vtr[5]*DL1[23]);

		FMR2[i][0]=c01*(Vtr[0]*DL2[0]+Vtr[1]*DL2[4]+Vtr[2]*DL2[8]+Vtr[3]*DL2[12]+Vtr[4]*DL2[16]+Vtr[5]*DL2[20]);
		FMR2[i][1]=c01*(Vtr[0]*DL2[1]+Vtr[1]*DL2[5]+Vtr[2]*DL2[9]+Vtr[3]*DL2[13]+Vtr[4]*DL2[17]+Vtr[5]*DL2[21]);
		FMR2[i][2]=c01*(Vtr[0]*DL2[2]+Vtr[1]*DL2[6]+Vtr[2]*DL2[10]+Vtr[3]*DL2[14]+Vtr[4]*DL2[18]+Vtr[5]*DL2[22]);
		FMR2[i][3]=c01*(Vtr[0]*DL2[3]+Vtr[1]*DL2[7]+Vtr[2]*DL2[11]+Vtr[3]*DL2[15]+Vtr[4]*DL2[19]+Vtr[5]*DL2[23]);

		FMR3[i][0]=c01*(Vtr[0]*DL3[0]+Vtr[1]*DL3[4]+Vtr[2]*DL3[8]+Vtr[3]*DL3[12]+Vtr[4]*DL3[16]+Vtr[5]*DL3[20]);
		FMR3[i][1]=c01*(Vtr[0]*DL3[1]+Vtr[1]*DL3[5]+Vtr[2]*DL3[9]+Vtr[3]*DL3[13]+Vtr[4]*DL3[17]+Vtr[5]*DL3[21]);
		FMR3[i][2]=c01*(Vtr[0]*DL3[2]+Vtr[1]*DL3[6]+Vtr[2]*DL3[10]+Vtr[3]*DL3[14]+Vtr[4]*DL3[18]+Vtr[5]*DL3[22]);
		FMR3[i][3]=c01*(Vtr[0]*DL3[3]+Vtr[1]*DL3[7]+Vtr[2]*DL3[11]+Vtr[3]*DL3[15]+Vtr[4]*DL3[19]+Vtr[5]*DL3[23]);
	}
	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

UINT WINAPI Maille::ColThreadProc(LPVOID pParam)
{
	Maille * pThis = reinterpret_cast<Maille*>( pParam );
	_ASSERTE( pThis != NULL );
	
	//	pThis->Run();
	while(1)

	{
			if(pThis->endColThread)
	_endthread();

		if(pThis->IsMaillePhantom)
		{
			
//			gstPoint phantomPos = pThis->MaillePhantom->getPosition_WC();
//			gstPoint phantomVel = pThis->MaillePhantom->getVelocity();
//			pThis->Curs_Pos[0]=0.3*phantomPos[0];
//			pThis->Curs_Pos[1]=0.3*phantomPos[1];
//			pThis->Curs_Pos[2]=0.3*phantomPos[2];
			
//			pThis->Curs_Velocity[0]=0.3*phantomVel[0];
//			pThis->Curs_Velocity[1]=0.3*phantomVel[1];
//			pThis->Curs_Velocity[2]=0.3*phantomVel[2];
		}
		else
		{
			int a=2;
		}
		
		//	pThis->collision(pThis->Curs_Pos[0],pThis->Curs_Pos[1],pThis->Curs_Pos[2]);
		Sleep(10);
	}
	return 1L;
}


UINT WINAPI Maille::CalThreadProc(LPVOID pParam)
{
	Maille * pThis = reinterpret_cast<Maille*>( pParam );
	_ASSERTE( pThis != NULL );

//	pThis->Run();
while(1)
{
	if(pThis->endCalThread) _endthread();

	pThis->CalculForces_et_Integration();

//	Sleep(1);
}
	return 1L;
} // end SocketThreadProc


void Maille::extraire_data()
{
	
	CString filename;
	filename=FilePath+".t";
	FILE *pfile;

	if( (pfile  = fopen( filename, "r" )) == NULL )
		MessageBox(0, filename,"le",0);		//		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf( pfile, "%d %d", &numVertices, &numTetra);//, &numTrianglePolygons );
	//getch();
	
	sommet=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		sommet[ll]=new double[3];

	tab_acc=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		tab_acc[ll]=new double[3];

	d_sommet1=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		d_sommet1[ll]=new double[3];

	d_sommet2=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		d_sommet2[ll]=new double[3];

	d_sommet3=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		d_sommet3[ll]=new double[3];

	d_sommet4=new TabDblPtr[numVertices];
	for(int ll=0;ll<numVertices;ll++)
		d_sommet4[ll]=new double[3];

	int  ll;
	vitesse=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		vitesse[ll]=new double[3];
	
	d_vitesse1=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		d_vitesse1[ll]=new double[3];
	
	d_vitesse2=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		d_vitesse2[ll]=new double[3];
	
	d_vitesse3=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		d_vitesse3[ll]=new double[3];
	
	d_vitesse4=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		d_vitesse4[ll]=new double[3];

	force=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		force[ll]=new double[3];

	gama=new TabDblPtr[numVertices]; 
	for(ll=0;ll<numVertices;ll++)
		gama[ll]=new double[3];

	forcevisco=new TabDblPtr[numVertices]; 
	for(ll=0;ll<numVertices;ll++)
		forcevisco[ll]=new double[3];

	acceleration_temp=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		acceleration_temp[ll]=new double[3];

	Collision_Deplacement=new TabDblPtr[numVertices];
	for(ll=0;ll<numVertices;ll++)
		Collision_Deplacement[ll]=new double[3];
	isCollision=new bool[numVertices];
	for(ll=0;ll<numVertices;ll++)
		isCollision[ll]=false;

	for(ll=0;ll<numVertices;ll++)
	{
		Collision_Deplacement[ll][0]=0.;
		Collision_Deplacement[ll][1]=0.;
		Collision_Deplacement[ll][2]=0.;
		vitesse[ll][0]=0.;
		vitesse[ll][1]=0.;
		vitesse[ll][2]=0.;
	}
	tetra=new TabIntPtr[numTetra];
	for(ll=0;ll<numTetra;ll++)
		tetra[ll]=new int[4];

	tetraLink=new TabIntPtr[numTetra];
	for(ll=0;ll<numTetra;ll++)
		tetraLink[ll]=new int[6];

	InitVolume=new double[numTetra];
	cteVolume=new double[numTetra];

	VoisinTetra=new TabIntPtr[numTetra];
	for(ll=0;ll<numTetra;ll++)
		VoisinTetra[ll]=new int[4];	
	
	for(int i=0; i<numVertices;i++)
	{
		fscanf( pfile, "%lf %lf %lf ",&sommet[i][0], &sommet[i][1], &sommet[i][2]);

		d_sommet1[i][0]=sommet[i][0];
		d_sommet1[i][1]=sommet[i][1];
		d_sommet1[i][2]=sommet[i][2];

		d_sommet2[i][0]=sommet[i][0];
		d_sommet2[i][1]=sommet[i][1];
		d_sommet2[i][2]=sommet[i][2];

		d_sommet3[i][0]=sommet[i][0];
		d_sommet3[i][1]=sommet[i][1];
		d_sommet3[i][2]=sommet[i][2];

		d_sommet4[i][0]=sommet[i][0];
		d_sommet4[i][1]=sommet[i][1];
		d_sommet4[i][2]=sommet[i][2];
	}

	int numMax=0;
	int i;
	for(i=0; i<numTetra; i++)
	{
		fscanf( pfile, "%d %d %d %d",&(tetra[i][0]), &(tetra[i][1]), &(tetra[i][2]),&(tetra[i][3]));
		for (int j=0;j<4;j++)
			if (tetra[i][j]>numMax) numMax=tetra[i][j];
	}
	// File handling for which indexes of tetrahedra start at 1 
	if (numMax==numVertices)
		for(i=0; i<numTetra; i++)
			for (int j=0;j<4;j++)
				tetra[i][j]--;

	fclose(pfile);
	
	filename=FilePath+".tetra.neu";
				
	if( (pfile  = fopen( filename, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf( pfile, "%d\n ", numVertices);
	for( i=0; i<numVertices;i++)
		fprintf(pfile, "%lf %lf %lf\n",sommet[i][0], sommet[i][1], sommet[i][2]);
	
	fclose(pfile);
}


void Maille::charger_surface()
{
	CString filename;
	filename=FilePath+".surface";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf( pfile, "%d ",&numTri);

	SurfaceTri=new TabIntPtr[numTri];
	for(int ll=0;ll<numTri;ll++)
		SurfaceTri[ll]=new int[3];	
	
	for(int i=0; i<numTri;i++)
	{
		fscanf( pfile, "%d %d %d ",&SurfaceTri[i][0],&SurfaceTri[i][1],&SurfaceTri[i][2]);
	}
	
	fclose(pfile);
}

void Maille::extraire_TetraVoisin()
{
	int flag=0;
	int i;
	int j;
	int k;
	
	for( i=0; i<numTetra;i++)
	{
		VoisinTetra[i][0]=-1;
		VoisinTetra[i][1]=-1;
		VoisinTetra[i][2]=-1;
		VoisinTetra[i][3]=-1;
	}
	
	for( i=0; i<numTetra-1;i++)
	{
		if (VoisinTetra[i][3]==-1)
		{
			
			for( j=i+1; j<numTetra;j++)
			{
				flag=0;
				if (VoisinTetra[j][3]==-1)
				{
					for( k=0; k<4;k++)
					{
						if ( (tetra[i][k]==tetra[j][0])||(tetra[i][k]==tetra[j][1])||(tetra[i][k]==tetra[j][2])||(tetra[i][k]==tetra[j][3])	)
							flag++;
					}
					
					
					if (flag==3)
					{
						flag=0;
						for( k=0; k<4;k++)
							if (VoisinTetra[i][k]==-1)
							{
								VoisinTetra[i][k]=j;
								break;
							}
							for( k=0; k<4;k++)
								if (VoisinTetra[j][k]==-1)
								{
									VoisinTetra[j][k]=i;
									break;
								} 	
					}
					
				}
			}
		}
	}	
		
		CString filename;
		filename=FilePath+".tetra";
		FILE *pfile;
		
		if( (pfile  = fopen( filename, "w" )) == NULL )
			printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
		else
			printf( "\n-> The file '%s' was opened.\n", filename );
		
		for( i=0; i<numTetra;i++)
			fprintf( pfile, "%d %d %d %d \n",tetra[i][0], tetra[i][1], tetra[i][2],tetra[i][3]);
		
		fclose(pfile);
		
		filename=FilePath+".voisin";
		
		if( (pfile  = fopen( filename, "w" )) == NULL )
			printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
		else
			printf( "\n-> The file '%s' was opened.\n", filename );
		
		for( i=0; i<numTetra;i++)
			fprintf( pfile, "%d %d %d %d \n",VoisinTetra[i][0], VoisinTetra[i][1], VoisinTetra[i][2],VoisinTetra[i][3]);
		int c=0;
		fclose(pfile);		
		
}


void Maille::extraire_surface()
{
	int i;
	CString filename;
	filename=FilePath+".tetsurface";
	FILE *pfile;
	
	numTri=0;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf( pfile, "%d ",&numtetSurface);
	
	CString filename1;
	filename1=FilePath+".temp";
	FILE *pfile1;
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	SurfaceTetra=new TabIntPtr[numtetSurface];
	for(int ll=0;ll<numtetSurface;ll++)
		SurfaceTetra[ll]=new int[4];

	for(i=0; i<numtetSurface;i++)
	{
		fscanf( pfile, "%d %d %d %d ",&SurfaceTetra[i][0],&SurfaceTetra[i][1],&SurfaceTetra[i][2],&SurfaceTetra[i][3]);
		
		if (
			!(TriDansTetra(tetra,SurfaceTetra[i][1],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][2]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][2],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][2]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][3],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][2]))
			)
		{
			numTri++;

			int A=tetra[(SurfaceTetra[i][0])][0], B=tetra[(SurfaceTetra[i][0])][1], C=tetra[(SurfaceTetra[i][0])][2];
			int D=tetra[(SurfaceTetra[i][0])][3];

			double DA[3]={sommet[A][0]-sommet[D][0],sommet[A][1]-sommet[D][1],sommet[A][2]-sommet[D][2]};
			double AB[3]={sommet[B][0]-sommet[A][0],sommet[B][1]-sommet[A][1],sommet[B][2]-sommet[A][2]};
			double AC[3]={sommet[C][0]-sommet[A][0],sommet[C][1]-sommet[A][1],sommet[C][2]-sommet[A][2]};
			//U.(V^W)=U1V2W3 + U2V3W1 +	U3V1W2 - U3V2W1 - U2V1W3 - U1V3W2
			if( (DA[0]*AB[1]*AC[2] + DA[1]*AB[2]*AC[0] + DA[2]*AB[0]*AC[1] - 
				DA[2]*AB[1]*AC[0] - DA[1]*AB[0]*AC[2] - DA[0]*AB[2]*AC[1]) > 0)
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][1],tetra[(SurfaceTetra[i][0])][2]);
			}
			else
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][2],tetra[(SurfaceTetra[i][0])][1]);
			}

		}
		
		if (
			!(TriDansTetra(tetra,SurfaceTetra[i][1],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][2],tetra[SurfaceTetra[i][0]][3]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][2],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][2],tetra[SurfaceTetra[i][0]][3]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][3],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][2],tetra[SurfaceTetra[i][0]][3]))
			)
		{
			numTri++;

			int A=tetra[(SurfaceTetra[i][0])][0], B=tetra[(SurfaceTetra[i][0])][2], C=tetra[(SurfaceTetra[i][0])][3];
			int D=tetra[(SurfaceTetra[i][0])][1];

			double DA[3]={sommet[A][0]-sommet[D][0],sommet[A][1]-sommet[D][1],sommet[A][2]-sommet[D][2]};
			double AB[3]={sommet[B][0]-sommet[A][0],sommet[B][1]-sommet[A][1],sommet[B][2]-sommet[A][2]};
			double AC[3]={sommet[C][0]-sommet[A][0],sommet[C][1]-sommet[A][1],sommet[C][2]-sommet[A][2]};
			//U.(V^W)=U1V2W3 + U2V3W1 +	U3V1W2 - U3V2W1 - U2V1W3 - U1V3W2
			if( (DA[0]*AB[1]*AC[2] + DA[1]*AB[2]*AC[0] + DA[2]*AB[0]*AC[1] - 
				DA[2]*AB[1]*AC[0] - DA[1]*AB[0]*AC[2] - DA[0]*AB[2]*AC[1]) > 0)
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][2],tetra[(SurfaceTetra[i][0])][3]);
			}
			else
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][3],tetra[(SurfaceTetra[i][0])][2]);
			}

		}
		
		if (
			!(TriDansTetra(tetra,SurfaceTetra[i][1],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][1]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][2],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][1]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][3],tetra[SurfaceTetra[i][0]][0],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][1]))
			)
		{
			numTri++;

			int A=tetra[(SurfaceTetra[i][0])][0], B=tetra[(SurfaceTetra[i][0])][3], C=tetra[(SurfaceTetra[i][0])][1];
			int D=tetra[(SurfaceTetra[i][0])][2];

			double DA[3]={sommet[A][0]-sommet[D][0],sommet[A][1]-sommet[D][1],sommet[A][2]-sommet[D][2]};
			double AB[3]={sommet[B][0]-sommet[A][0],sommet[B][1]-sommet[A][1],sommet[B][2]-sommet[A][2]};
			double AC[3]={sommet[C][0]-sommet[A][0],sommet[C][1]-sommet[A][1],sommet[C][2]-sommet[A][2]};
			//U.(V^W)=U1V2W3 + U2V3W1 +	U3V1W2 - U3V2W1 - U2V1W3 - U1V3W2
			if( (DA[0]*AB[1]*AC[2] + DA[1]*AB[2]*AC[0] + DA[2]*AB[0]*AC[1] - 
				DA[2]*AB[1]*AC[0] - DA[1]*AB[0]*AC[2] - DA[0]*AB[2]*AC[1]) > 0)
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][3],tetra[(SurfaceTetra[i][0])][1]);
			}
			else
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][0],tetra[(SurfaceTetra[i][0])][1],tetra[(SurfaceTetra[i][0])][3]);
			}
		}
		
		if (
			!(TriDansTetra(tetra,SurfaceTetra[i][1],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][2]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][2],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][2]))&&
			!(TriDansTetra(tetra,SurfaceTetra[i][3],tetra[SurfaceTetra[i][0]][1],tetra[SurfaceTetra[i][0]][3],tetra[SurfaceTetra[i][0]][2]))
			)
		{
			numTri++;

			int A=tetra[(SurfaceTetra[i][0])][1], B=tetra[(SurfaceTetra[i][0])][3], C=tetra[(SurfaceTetra[i][0])][2];
			int D=tetra[(SurfaceTetra[i][0])][0];

			double DA[3]={sommet[A][0]-sommet[D][0],sommet[A][1]-sommet[D][1],sommet[A][2]-sommet[D][2]};
			double AB[3]={sommet[B][0]-sommet[A][0],sommet[B][1]-sommet[A][1],sommet[B][2]-sommet[A][2]};
			double AC[3]={sommet[C][0]-sommet[A][0],sommet[C][1]-sommet[A][1],sommet[C][2]-sommet[A][2]};
			//U.(V^W)=U1V2W3 + U2V3W1 +	U3V1W2 - U3V2W1 - U2V1W3 - U1V3W2
			if( (DA[0]*AB[1]*AC[2] + DA[1]*AB[2]*AC[0] + DA[2]*AB[0]*AC[1] - 
				DA[2]*AB[1]*AC[0] - DA[1]*AB[0]*AC[2] - DA[0]*AB[2]*AC[1]) > 0)
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][1],tetra[(SurfaceTetra[i][0])][3],tetra[(SurfaceTetra[i][0])][2]);

			}
			else
			{
				fprintf( pfile1, "%d %d %d\n",tetra[(SurfaceTetra[i][0])][1],tetra[(SurfaceTetra[i][0])][2],tetra[(SurfaceTetra[i][0])][3]);
			}
			
		}
	}
	
	fclose(pfile);
	
	fclose(pfile1);
	
	if( (pfile1  = fopen( filename1, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1);	

	CString filename2;
	filename2=FilePath+".surface";
	FILE *pfile2;
	
	if( (pfile2  = fopen( filename2, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename2);
	else
		printf( "\n-> The file '%s' was opened.\n", filename2 );
	
	fprintf( pfile2, "%d\n",numTri);

	int a,b,c;
	for(i=0;i<numTri;i++)
	{
		fscanf(pfile1,"%d %d %d",&a,&b,&c);
		fprintf(pfile2,"%d %d %d\n",a,b,c);
	}
	
	fclose(pfile1);
	fclose(pfile2);
}

void Maille::extraire_tetsurface()
{
	int i;
	CString filename;
	filename=FilePath+".voisin";
	FILE *pfile;
	
	CString filename1;
	filename1=FilePath+".tetsurface";
	FILE *pfile1;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1 );
	
	numtetSurface=0;
	
	for( i=0; i<numTetra;i++)
	{
		fscanf( pfile, "%d %d %d %d",&(VoisinTetra[i][0]), &(VoisinTetra[i][1]), &(VoisinTetra[i][2]),&(VoisinTetra[i][3]));
		if (VoisinTetra[i][3]==-1)
			numtetSurface++;
	}
	
	fprintf( pfile1, "%d \n",numtetSurface);
	
	for( i=0; i<numTetra;i++)
	{
		if (VoisinTetra[i][3]==-1)
			fprintf( pfile1, "%d %d %d %d\n",i,VoisinTetra[i][0],VoisinTetra[i][1],VoisinTetra[i][2]);
	}

	fclose(pfile);
	fclose(pfile1);
		
}

void Maille::dessine_maille()
{
	clock_t start, finish;
	start = clock();

	glScalef((float)10.0*(float)scale,(float)10.0*(float)scale,(float)10.0*(float)scale);
	glTranslatef((float)xTranslate,(float)yTranslate,(float)zTranslate);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	// Mesh surface triangles
	glBegin(GL_TRIANGLES);
	for(int i=0;i<numTri;i++)
	{	
		glColor3f( 1.0f, 0.0f, 0.0f );
		glVertex3d(sommet[(SurfaceTri[i][0])][0], sommet[(SurfaceTri[i][0])][1], sommet[(SurfaceTri[i][0])][2]);
		glColor3f( 0.0f, 1.0f, 0.0f );
		glVertex3d(sommet[(SurfaceTri[i][1])][0], sommet[(SurfaceTri[i][1])][1], sommet[(SurfaceTri[i][1])][2]);
		glColor3f( 0.0f, 0.0f, 1.0f );
		glVertex3d(sommet[(SurfaceTri[i][2])][0], sommet[(SurfaceTri[i][2])][1], sommet[(SurfaceTri[i][2])][2]);	
	}
	glEnd();

	if(!Indent)
	{
		// Sphere for interaction
		xSphere=Curs_Pos[0];
		ySphere=Curs_Pos[1];
		zSphere=Curs_Pos[2];
		glPushMatrix();
		GLUquadricObj *quadObj;
		quadObj = gluNewQuadric ();
			glColor3f( 0.5f, 0.5f, 0.0f );
			glTranslated(xSphere,ySphere,zSphere);
			gluSphere(quadObj,2.0,10,10);
		glPopMatrix();

		// Plan horizontal inférieur
		glBegin(GL_POLYGON);
		glColor3f( 1.0f, 1.0f, 1.0f );

		glVertex3d(-200.,-30,-200);    //glVertex3d(-200.,-150,-200);
		glVertex3d(-200.,-30,200);     //glVertex3d(-200.,-150,200); 
		glVertex3d(200.,-30,200);      //glVertex3d(200.,-150,200);
		glVertex3d(200.,-30,-200);     //glVertex3d(200.,-150,-200); 
		glEnd();
	}

	// Indent: compression plans
	if(Indent)
	{
		glPushMatrix();
		GLUquadricObj *quadObj5;
		quadObj5 = gluNewQuadric ();
		glColor3f( 0.8f, 0.5f, 0.0f );
		glTranslated(0,0,yIndent_sup);
		//glTranslated(0,yIndent_sup,0);
		//glRotatef(-90, 1.0, 0.0, 0.0);
		gluCylinder(quadObj5, 1,1,2*force_indent,15,5);
		glPopMatrix();

		// Bottom horizontal plane
		glBegin(GL_POLYGON);
		glColor3f( 1.0f, 1.0f, 1.0f );

		glVertex3d(-50.,-50.,yIndent_inf);
		glVertex3d(-50.,50.,yIndent_inf);
		glVertex3d(50.,50.,yIndent_inf);
		glVertex3d(50.,-50.,yIndent_inf);
		glEnd();

		// Up horizontal plane
		glBegin(GL_POLYGON);
		glColor3f( 1.0f, 1.0f, 1.0f );

		glVertex3d(-50.,-50.,yIndent_sup);
		glVertex3d(-50.,50.,yIndent_sup);
		glVertex3d(50.,50.,yIndent_sup);
		glVertex3d(50.,-50.,yIndent_sup);
		glEnd();
	}
	
	finish = clock();
	
	temps_dessin = (float)(finish - start) * 1000 / CLOCKS_PER_SEC;    	  
}

/////////////////

bool Maille::PointDansTetra(int **Tabtetra,int ntetra, int npoint)
{
	for(int i=0; i<4; i++)
		if (npoint==Tabtetra[ntetra][i])
			return true;
		return false;
}

bool Maille::PointDansTri(int **Tabtri,int ntri, int npoint)
{
	for(int i=0; i<3; i++)
		if (npoint==Tabtri[ntri][i])
			return true;
		return false;
}

bool Maille::TriDansTetra(int **Tabtetra,int ntetra,int **Tabtri,int ntri)
{
	for(int i=0; i<3; i++)
		if (!PointDansTetra(Tabtetra,ntetra,Tabtri[ntri][i]))
			return false;
		return true;
}

bool Maille::TriDansTetra(int **Tabtetra,int ntetra,int triPoint1,int triPoint2,int triPoint3)
{
	if (ntetra==-1)
		return false;
	
	return
		(
		PointDansTetra(Tabtetra,ntetra,triPoint1)&&
		PointDansTetra(Tabtetra,ntetra,triPoint2)&&
		PointDansTetra(Tabtetra,ntetra,triPoint3)
		);
}

void Maille::extraire_liaisons()
{
	int **TempLiaison;
	TempLiaison=new TabIntPtr[6*numTetra];
	for(int ll=0;ll<(6*numTetra);ll++)
		TempLiaison[ll]=new int[2];
	
	double *Somme_Vol_Ass;
	Somme_Vol_Ass=new double[6*numTetra];

	CString filename;
	filename=FilePath+".Volum";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	for(int i=0;i<numTetra;i++)
	{
		fscanf(pfile,"%lf",&(Somme_Vol_Ass[i*6]));
		Somme_Vol_Ass[i*6+1]=Somme_Vol_Ass[i*6];
		Somme_Vol_Ass[i*6+2]=Somme_Vol_Ass[i*6];
		Somme_Vol_Ass[i*6+3]=Somme_Vol_Ass[i*6];
		Somme_Vol_Ass[i*6+4]=Somme_Vol_Ass[i*6];
		Somme_Vol_Ass[i*6+5]=Somme_Vol_Ass[i*6];

		TempLiaison[i*6][0]=tetra[i][0];
		TempLiaison[i*6][1]=tetra[i][1];
		TempLiaison[i*6+1][0]=tetra[i][0];
		TempLiaison[i*6+1][1]=tetra[i][2];
		TempLiaison[i*6+2][0]=tetra[i][0];
		TempLiaison[i*6+2][1]=tetra[i][3];
		TempLiaison[i*6+3][0]=tetra[i][1];
		TempLiaison[i*6+3][1]=tetra[i][2];
		TempLiaison[i*6+4][0]=tetra[i][1]; 
		TempLiaison[i*6+4][1]=tetra[i][3];
		TempLiaison[i*6+5][0]=tetra[i][2];
		TempLiaison[i*6+5][1]=tetra[i][3];
	}
	int j,i;
	numLiaison=0;

	for (i=0; i<(6*numTetra);i++)
	{
		if(TempLiaison[i][0]!=-1)
		{
			numLiaison++;
			for(j=i+1;j<6*numTetra;j++)
			{
				if(TempLiaison[j][0]!=-1)
				{
					if (
						((TempLiaison[i][0]==TempLiaison[j][0]) && (TempLiaison[i][1]==TempLiaison[j][1]))||
						((TempLiaison[i][0]==TempLiaison[j][1]) && (TempLiaison[i][1]==TempLiaison[j][0]))
						)
					{
						TempLiaison[j][0]=TempLiaison[j][1]=-1;
						Somme_Vol_Ass[i]+=Somme_Vol_Ass[j];
					}
				}
			}
		}
	}
				
	CString mnumLiaison;
	
	mnumLiaison.Format(_T("nombre des Liaisons: %d\n"), numLiaison);
	
	CString filename1;
	filename1=FilePath+".liaison";
	FILE *pfile1;
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1 );
	
	fprintf(pfile1,"%d\n",numLiaison);
	
	for (i=0; i<6*numTetra;i++)
	{
		if (TempLiaison[i][0]!=-1)
			fprintf(pfile1,"%d %d %lf\n",TempLiaison[i][0],TempLiaison[i][1],Somme_Vol_Ass[i]);
	}
	
	fclose(pfile1);
	fclose(pfile);
	delete(Somme_Vol_Ass);

	for(int lll=0;lll<(6*numTetra);lll++)
		delete(TempLiaison[lll]);
	delete(TempLiaison);
}


void Maille::charger_liaisons()
// Loading the table of links
// Initialisation : calculating initial lengths, determining stiffness
{
	CString filename;
	filename=FilePath+".liaison";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );

	CString filename1;
	filename1=FilePath+".liaison_raideur";
	FILE *pfile1;
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1 );
	
	fscanf(pfile,"%d",&numLiaison);
	fprintf(pfile1,"%d\n",numLiaison);

	clock_t start, finish;
	start = clock();
	
	LinkExt=new TabIntPtr[numLiaison]; // Indexes of the links between vertices
	for(int i=0;i<numLiaison;i++)
		LinkExt[i]=new int[2]; 

	LinkVect=new TabDblPtr[numLiaison]; // Vectors of links between vertices
	for(int i=0;i<numLiaison;i++)
		LinkVect[i]=new double[3];

	LinkNormVect=new TabDblPtr[numLiaison]; // Vectors of normalized links between vertices
	for(int i=0;i<numLiaison;i++)
		LinkNormVect[i]=new double[3];

	Raideur=new double[numLiaison];
	LinkInit=new double[numLiaison];
	LinkInit2=new double[numLiaison];
	LinkLength=new double[numLiaison];
	LinkLength2=new double[numLiaison];
	deltaLong=new double[numLiaison];
	deltaLong2=new double[numLiaison];
	
	double tempooo_Vol;
	int ext0,ext1,i;
	for( i=0;i<numLiaison;i++)
	{
		fscanf(pfile,"%d %d %lf",&ext0,&ext1,&tempooo_Vol);
	
		LinkExt[i][0]=ext0;
		LinkExt[i][1]=ext1;	

		for (int j=0; j<3; j++)
			LinkVect[i][j]=sommet[ext0][j]-sommet[ext1][j];
		
		LinkLength2[i]=NORM2(LinkVect[i]);
		LinkInit2[i]=LinkLength2[i];
		LinkInit[i]=sqrt(LinkLength2[i]);
		LinkLength[i]=LinkInit[i];

		for (int j=0; j<3; j++)
			LinkNormVect[i][j]=LinkVect[i][j]/LinkLength[i];

		Raideur[i]=young*tempooo_Vol/LinkLength2[i];

		fprintf(pfile_acq,"Liaison %i : %e\n",i,LinkLength2[i]);

		fprintf(pfile1,"%d %d %lf %lf %lf\n",ext0,ext1,LinkInit[i],tempooo_Vol,Raideur[i]);

	}
	finish = clock();
	double duration;
	duration = (double)(finish - start) / CLOCKS_PER_SEC;	
	CString mDuration;
	mDuration.Format(_T("chargement des liaisons: %lf\n"), duration);
	fclose(pfile);
	fclose(pfile1);
}

void Maille::extraire_SomVoisin()
{
	CString filename;
	filename=FilePath+".liaison";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf(pfile,"%d",&numLiaison);
	
	int **TempLiaison;
	TempLiaison=new TabIntPtr[numLiaison];
	for(int ll=0;ll<numLiaison;ll++)
		TempLiaison[ll]=new int[2];
	double tempoVolume;
	for(int i=0;i<numLiaison;i++)
		fscanf(pfile,"%d %d %lf",&TempLiaison[i][0],&TempLiaison[i][1],&tempoVolume);
	fclose(pfile);
	
	CString filename1;
	filename1=FilePath+".liaisonAss";
	FILE *pfile1;
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1 );
	
	int i;
	TnumLiaisonAss=new int[numVertices];
	for(i=0;i<numVertices;i++)
		TnumLiaisonAss[i]=0;
	LiaisonAss=new TabIntPtr[numVertices];
	
	int tempLiaisonAss[1500];
	
	for(i=0;i<numVertices;i++)
	{
		for(int j=0;j<numLiaison;j++)
		{
			if( (i==TempLiaison[j][0])||(i==TempLiaison[j][1]))
			{
				tempLiaisonAss[TnumLiaisonAss[i]]=j;
				TnumLiaisonAss[i]++;
			}
		}
			
		fprintf(pfile1,"%d",TnumLiaisonAss[i]);
		for(int k=0;k<TnumLiaisonAss[i];k++)
			fprintf(pfile1," %d",tempLiaisonAss[k]);
		fprintf(pfile1,"\n");		
	}
		
	fclose(pfile1);
	
}


void Maille::charger_vertices()
{
	CString filename;
	filename=FilePath+".liaisonAss";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename);
}


void Maille::charger_LiaisonAss()
{
	CString filename;
	filename=FilePath+".liaisonAss";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename);
	
	TnumLiaisonAss=new int[numVertices];
	LiaisonAss=new TabIntPtr[numVertices];

	int tempNumLiaisAss;

	for(int i=0; i<numVertices;i++)
	{
		fscanf(pfile,"%d",&tempNumLiaisAss);
		TnumLiaisonAss[i]=tempNumLiaisAss;
		LiaisonAss[i] = new int[tempNumLiaisAss];
		for(int j=0;j<tempNumLiaisAss;j++)
			fscanf(pfile,"%d",&(LiaisonAss[i][j]));
	}
}

UINT Maille::Col_Detect(LPVOID pParam)	
{
return 0;
}

void  Maille::extraire_SP()
{
	int **Temp_SurfaceTri;
	Temp_SurfaceTri=new TabIntPtr[numTri];
	for(int ll=0;ll<numTri;ll++)
		Temp_SurfaceTri[ll]=new int[3];	

	for(int kk=0;kk<numTri;kk++)
	{
		Temp_SurfaceTri[kk][0]=SurfaceTri[kk][0];
		Temp_SurfaceTri[kk][1]=SurfaceTri[kk][1];
		Temp_SurfaceTri[kk][2]=SurfaceTri[kk][2];
	}
	
	for(int i=0;i<numTri-1;i++)
	{
		for(int j=i+1;j<numTri;j++)
		{
			for(int k=0;k<3;k++)
			{
				for(int l=0;l<3;l++)
				{
					if(Temp_SurfaceTri[i][k]==Temp_SurfaceTri[j][l])
					{
						Temp_SurfaceTri[j][l]=-1;
					}
				}
			}
		}
	}
	int temp_numSP=0;
	int i;
	for(i=0;i<numTri;i++)
	{
		for(int j=0;j<3;j++)
		{
			if(Temp_SurfaceTri[i][j]!=-1)
				temp_numSP++;
		}
	}

	CString filename;
	filename=FilePath+".SP";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fprintf( pfile, "%d\n",temp_numSP);

	for(i=0;i<numTri;i++)
	{
		for(int j=0;j<3;j++)
		{
			if(Temp_SurfaceTri[i][j]!=-1)
				fprintf(pfile,"%d\n",Temp_SurfaceTri[i][j]);
		}
	}
		
	fclose(pfile);
	for(int ll=0;ll<numTri;ll++)
		delete Temp_SurfaceTri[ll];
}

void  Maille::charger_SP()
{
	CString filename;
	filename=FilePath+".SP";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	fscanf( pfile, "%d\n",&numSP);

	SurfacePoint=new int[numSP];

	for(int i=0; i<numSP;i++)
		fscanf(pfile,"%d",&SurfacePoint[i]);
}

void  Maille::old_Tri_collision(double x,double y,double z)
{
#if 1
	
	double rayon_2=rayon*rayon;
	int tempnbCol_Tri=-1;
	nbCol_Center=0;

	for(int i=0;i<numTri;i++)
	{		
		double A[3]={sommet[(SurfaceTri[i][0])][0],sommet[(SurfaceTri[i][0])][1],sommet[(SurfaceTri[i][0])][2]};
		double B[3]={sommet[(SurfaceTri[i][1])][0],sommet[(SurfaceTri[i][1])][1],sommet[(SurfaceTri[i][1])][2]};
		double C[3]={sommet[(SurfaceTri[i][2])][0],sommet[(SurfaceTri[i][2])][1],sommet[(SurfaceTri[i][2])][2]};
		
		double AN=(B[1]-A[1])*(C[2]-A[2])-(C[1]-A[1])*(B[2]-A[2]);
		double BN=(B[2]-A[2])*(C[0]-A[0])-(C[2]-A[2])*(B[0]-A[0]);
		double CN=(B[0]-A[0])*(C[1]-A[1])-(C[0]-A[0])*(B[1]-A[1]);
		double DN=-(AN*A[0]+BN*A[1]+CN*A[2]);
		double distance_au_plan_2=(AN*x+BN*y+CN*z+DN)*(AN*x+BN*y+CN*z+DN)/(AN*AN+BN*BN+CN*CN);
		
		if (distance_au_plan_2<rayon_2)
		{
			
			double AB[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};
			double AC[3]={C[0]-A[0],C[1]-A[1],C[2]-A[2]};
			
			double PA[3]={A[0]-x,A[1]-y,A[2]-z};
			double PB[3]={B[0]-x,B[1]-y,B[2]-z};
			double PC[3]={C[0]-x,C[1]-y,C[2]-z};
			
			double AB_PA=AB[0]*PA[0]+AB[1]*PA[1]+AB[2]*PA[2];
			double AB_PB=AB[0]*PB[0]+AB[1]*PB[1]+AB[2]*PB[2];
			double AB_PC=AB[0]*PC[0]+AB[1]*PC[1]+AB[2]*PC[2];
			double AC_PA=AC[0]*PA[0]+AC[1]*PA[1]+AC[2]*PA[2];
			double AC_PB=AC[0]*PB[0]+AC[1]*PB[1]+AC[2]*PB[2];
			double AC_PC=AC[0]*PC[0]+AC[1]*PC[1]+AC[2]*PC[2];
			
			bool C1=(  ((AB_PA)*(AC_PB)-(AB_PB)*(AC_PA))  >=0 );
			bool C2=(  ((AB_PB)*(AC_PC)-(AB_PC)*(AC_PB))  >=0 );
			bool C3=(  ((AB_PC)*(AC_PA)-(AB_PA)*(AC_PC))  >=0 );
			if( C1 && C2 && C3)
			{
				tempnbCol_Tri++;
				Col_Tri[tempnbCol_Tri]=i;
				//calculating the projection point
				double t_proj=-(AN*x+BN*y+CN*z+DN)/(AN*AN+BN*BN+CN*CN);
				Col_Point_Proj[tempnbCol_Tri][0]=t_proj*AN+x;
				Col_Point_Proj[tempnbCol_Tri][1]=t_proj*BN+y;
				Col_Point_Proj[tempnbCol_Tri][2]=t_proj*CN+z;
				//calculating the expected center
				double t_centre=rayon/sqrt(AN*AN+BN*BN+CN*CN); 
				if(((AN*t_centre+Col_Point_Proj[tempnbCol_Tri][0]-sommet[138][0])*(AN*t_centre+Col_Point_Proj[tempnbCol_Tri][0]-sommet[138][0])+
					(BN*t_centre+Col_Point_Proj[tempnbCol_Tri][1]-sommet[138][1])*(BN*t_centre+Col_Point_Proj[tempnbCol_Tri][1]-sommet[138][1])+
					(CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2]-sommet[138][2])*(CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2]-sommet[138][2])) >
					((-AN*t_centre+Col_Point_Proj[tempnbCol_Tri][0]-sommet[138][0])*(-AN*t_centre+Col_Point_Proj[tempnbCol_Tri][0]-sommet[138][0])+
					(-BN*t_centre+Col_Point_Proj[tempnbCol_Tri][1]-sommet[138][1])*(-BN*t_centre+Col_Point_Proj[tempnbCol_Tri][1]-sommet[138][1])+
					(-CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2]-sommet[138][2])*(-CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2]-sommet[138][2])) 
					)
				{
					Col_Center[0][0] = AN*t_centre + Col_Point_Proj[tempnbCol_Tri][0];
					Col_Center[0][1] = BN*t_centre + Col_Point_Proj[tempnbCol_Tri][1];
					Col_Center[0][2]=CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2];
				}
				else
				{
					Col_Center[0][0]=-AN*t_centre+Col_Point_Proj[tempnbCol_Tri][0];
					Col_Center[0][1]=-BN*t_centre+Col_Point_Proj[tempnbCol_Tri][1];
					Col_Center[0][2]=-CN*t_centre+Col_Point_Proj[tempnbCol_Tri][2];
				}
		
				ForceResultante[0]=Col_Center[0][0];
				ForceResultante[1]=Col_Center[0][1];
				ForceResultante[2]=Col_Center[0][2];

				int ccccc=2;
			}
			else
			{
				//bool C4=AP.AC   > 0
				bool C4=(  AC_PA  <=0 );
				//bool C5=CP.CA=PC.AC
				bool C5=( AC_PC >=0);
				//bool C6=AP.AB=-AB.PA   >=0 => AB.PA <=0
				bool C6=(AB_PA <=0);
				//bool C7=BP.BA
				bool C7=(AB_PB >=0);
				double BC[3]={C[0]-B[0],C[1]-B[1],C[2]-B[2]};
				
				double BC_PB=BC[0]*PB[0]+BC[1]*PB[1]+BC[2]*PB[2];
				double BC_PC=BC[0]*PC[0]+BC[1]*PC[1]+BC[2]*PC[2];
				
				//bool C8=BP.BC=-BC.PB
				bool C8=(BC_PB<=0);
				//bool C9=CP.CB=BC.PC
				bool C9=(BC_PC>=0);
				
				if(!C3)
				{
					if (C4 && C5)
					{
						// distance_P_AC_2= AP²-AH² with H projection of P on AC , AH=(AP.AC)/|AC|
						double distance_P_AC_2=PA[0]*PA[0]+PA[1]*PA[1]+PA[2]*PA[2]-(AC_PA*AC_PA)/(AC[0]*AC[0]+AC[1]*AC[1]+AC[2]*AC[2]);
						if (distance_P_AC_2 <=rayon_2)
						{
							tempnbCol_Tri++;
							Col_Tri[tempnbCol_Tri]=i;
						double t_proj=(AC[0]*(x-A[0])+AC[1]*(y-A[1])+AC[2]*(z-A[2]))/(AC[0]*AC[0]+AC[1]*AC[1]+AC[2]*AC[2]);
						Col_Point_Proj[tempnbCol_Tri][0]=t_proj*AC[0]+A[0];
						Col_Point_Proj[tempnbCol_Tri][1]=t_proj*AC[1]+A[1];
						Col_Point_Proj[tempnbCol_Tri][2]=t_proj*AC[2]+A[2];
						}
					}
				}
				if(!C1)
				{
					if (C6 && C7)
					{
						// distance_P_AB_2= AP²-AH² with H projection of P on AB , AH=(AP.AB)/|AB|
						double distance_P_AB_2=PA[0]*PA[0]+PA[1]*PA[1]+PA[2]*PA[2]-(AB_PA*AB_PA)/(AB[0]*AB[0]+AB[1]*AB[1]+AB[2]*AB[2]);
						if (distance_P_AB_2 <=rayon_2)
						{
							tempnbCol_Tri++;
							Col_Tri[tempnbCol_Tri]=i;
							double t_proj=(AB[0]*(x-A[0])+AB[1]*(y-A[1])+AB[2]*(z-A[2]))/(AB[0]*AB[0]+AB[1]*AB[1]+AB[2]*AB[2]);
							Col_Point_Proj[tempnbCol_Tri][0]=t_proj*AB[0]+A[0];
							Col_Point_Proj[tempnbCol_Tri][1]=t_proj*AB[1]+A[1];
							Col_Point_Proj[tempnbCol_Tri][2]=t_proj*AB[2]+A[2];
						}
						
					}
				}
				if (!C2)
				{
					if(C8 && C9)
					{
						// distance_P_CB_2= CP²-CH² with H projection of P on CB , CH=(CP.CB)/|CB|
						double distance_P_CB_2=PC[0]*PC[0]+PC[1]*PC[1]+PC[2]*PC[2]-(BC_PC*BC_PC)/(BC[0]*BC[0]+BC[1]*BC[1]+BC[2]*BC[2]);
						if (distance_P_CB_2 <=rayon_2)
						{
							tempnbCol_Tri++;
							Col_Tri[tempnbCol_Tri]=i;
							double t_proj=(BC[0]*(x-C[0])+BC[1]*(y-C[1])+BC[2]*(z-C[2]))/(BC[0]*BC[0]+BC[1]*BC[1]+BC[2]*BC[2]);
							Col_Point_Proj[tempnbCol_Tri][0]=t_proj*BC[0]+C[0];
							Col_Point_Proj[tempnbCol_Tri][1]=t_proj*BC[1]+C[1];
							Col_Point_Proj[tempnbCol_Tri][2]=t_proj*BC[2]+C[2];
						}
					}
				}
				
				if( ((!C3)&&(!C4))||( (!C1)&&(!C6)))		//d=d(P,A)
				{
					double distance_P_A_2=PA[0]*PA[0]+PA[1]*PA[1]+PA[2]*PA[2];
					if (distance_P_A_2 <=rayon_2)
					{
						tempnbCol_Tri++;
						Col_Tri[tempnbCol_Tri]=i;
						Col_Point_Proj[tempnbCol_Tri][0]=A[0];
						Col_Point_Proj[tempnbCol_Tri][1]=A[1];
						Col_Point_Proj[tempnbCol_Tri][2]=A[2];
					}
				}
				if( ((!C1)&&(!C7))||( (!C2)&&(!C8)))		//d=d(P,B)
				{
					double distance_P_B_2=PB[0]*PB[0]+PB[1]*PB[1]+PB[2]*PB[2];
					if (distance_P_B_2 <=rayon_2)
					{											
						tempnbCol_Tri++;
						Col_Tri[tempnbCol_Tri]=i;
						Col_Point_Proj[tempnbCol_Tri][0]=B[0];
						Col_Point_Proj[tempnbCol_Tri][1]=B[1];
						Col_Point_Proj[tempnbCol_Tri][2]=B[2];
					}
				}
				if( ((!C2)&&(!C9))||( (!C3)&&(!C5)))		//d=d(P,C)
				{
					double distance_P_C_2=PC[0]*PC[0]+PC[1]*PC[1]+PC[2]*PC[2];
					if (distance_P_C_2 <=rayon_2)
					{
						tempnbCol_Tri++;
						Col_Tri[tempnbCol_Tri]=i;
						Col_Point_Proj[tempnbCol_Tri][0]=C[0];
						Col_Point_Proj[tempnbCol_Tri][1]=C[1];
						Col_Point_Proj[tempnbCol_Tri][2]=C[2];
					}
				}
			}			
		}			
	}
	nbCol_Tri=tempnbCol_Tri;

#endif

	return;
}

void  Maille::collision(double x,double y,double z)
{
	double rayon_2=rayon*rayon;
	
	int tempsommetCollision;

	double xp,yp,zp;
	for(int i=0;i<numSP;i++)
	{
		//calculating the distance between the surface point and the radius of the sphere
		xp=sommet[SurfacePoint[i]][0];
		yp=sommet[SurfacePoint[i]][1];
		zp=sommet[SurfacePoint[i]][2];
		
		double SPdistance_2=(x -xp )*(x -xp)+(y - yp)*(y -yp)+ (z - zp)*(z -zp);
		if(SPdistance_2<=rayon_2)
		{
			tempsommetCollision=SurfacePoint[i];

			double SPdistance=sqrt(SPdistance_2);
			double t=rayon/SPdistance;
			double xc=(xp-x)*t+x;
			double yc=(yp-y)*t+y;
			double zc=(zp-z)*t+z;

			Collision_Deplacement[SurfacePoint[i]][0]=(xc-xp);
			Collision_Deplacement[SurfacePoint[i]][1]=(yc-yp);
			Collision_Deplacement[SurfacePoint[i]][2]=(zc-zp);

			isCollision[SurfacePoint[i]]=true;		
		}
		else
		{
			Collision_Deplacement[SurfacePoint[i]][0]=0.;
			Collision_Deplacement[SurfacePoint[i]][1]=0.;
			Collision_Deplacement[SurfacePoint[i]][2]=0.;
			isCollision[SurfacePoint[i]]=false;
		}
	}
	sommetCollision=tempsommetCollision;
}

void  Maille::SetMaillePhantom(void *phantom)
{
	IsMaillePhantom=true;
}

///////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculLongLiaisons(int numLiaison, int **LinkExt, double *deltaLong2)
// Calculation of link lengths
// In : number of links, table of point index, table of summits	
// Out :  table of difference between length squared and initial length squared 
{
	for(int i=0; i<numLiaison; i++)
	{
		//RLiaison[i].Calc_Force();
		Lext0=LinkExt[i][0];
		Lext1=LinkExt[i][1];

		LongX=sommet[Lext0][0]-sommet[Lext1][0];
		LongY=sommet[Lext0][1]-sommet[Lext1][1];
		LongZ=sommet[Lext0][2]-sommet[Lext1][2];
		LinkLength2[i]=LongX*LongX+LongY*LongY+LongZ*LongZ;
		Long=sqrt(LinkLength2[i]);
		LinkLength[i]=Long;
		
		deltaLong[i]=LinkLength[i]-LinkInit[i];
		deltaLong2[i]=LinkLength2[i]-LinkInit2[i];
	}

	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////


int Maille::CalculLiaisons(int numLiaison, int **LinkExt, TabDblPtr *sommet,
						   double **LinkNormVect, double *deltaLong, double *deltaLong2)
// Calculation of link lengths
// In : number of links, table of point index, table of summits	
// Out :  table of normalized links vectors
//			 table of difference between length and initial length
//			 table of difference between length squared and initial length squared
{
	for(int i=0; i<numLiaison; i++)
	{
		Lext0=LinkExt[i][0];
		Lext1=LinkExt[i][1];

		for (int j=0; j<3; j++)
			LinkVect[i][j]=sommet[Lext0][j]-sommet[Lext1][j];

		LinkLength2[i]=NORM2(LinkVect[i]);
		Long=sqrt(LinkLength2[i]);
		LinkLength[i]=Long;

		for (int j=0; j<3; j++)
			LinkNormVect[i][j]=LinkVect[i][j]/LinkLength[i];

		deltaLong[i]=LinkLength[i]-LinkInit[i];
		deltaLong2[i]=LinkLength2[i]-LinkInit2[i];
	}

	return OK;
}


///////////////////////////////////////////////////////////////////////////////////////////


int Maille::CalculForcesElastiques_MS3D(int numVertices, double **sommet, 
										int numLiaison, int **LinkExt,
										int *TnumLiaisonAss, int **LiaisonAss,
										double **force)
// Calculation of elastic forces for 3D mass-spring
// In :	number of vertices, 
//				vertice positions,
//				number of links, indice of links,
//				number of links associated with vertices and tables of related links
// Out : Force field on vertices (initialized in zero for input)
{
	int i,j,k;
	double Const;

	CalculLiaisons(numLiaison,LinkExt,sommet,LinkNormVect,deltaLong,deltaLong2);

    for(i=0;i<numVertices;i++)
    {
		for(j=0;j<TnumLiaisonAss[i];j++)
        {
			k=LiaisonAss[i][j]; 
			if(i==LinkExt[k][0]) 
				Const = -Raideur[k]*deltaLong[k];
			else
				Const = Raideur[k]*deltaLong[k];
			force[i][0]+=Const*LinkNormVect[k][0];
			force[i][1]+=Const*LinkNormVect[k][1];
			force[i][2]+=Const*LinkNormVect[k][2];
	    }

	}

	// Add forces relating to volume
	CalculAjoutForcesVolume_MS3D(sommet,numTetra,tetra,InitVolume,raideurVolume,force);

	return OK;
}


///////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculForcesHyperElastiques_STVK(int numVertices, TabDblPtr *sommet,
											 int numTetra, int **tetra,
											 double *InitVolume,
											 TabDblPtr *FM1, TabDblPtr *FM2, TabDblPtr *FM3, double **force)
// Calculate hyperelastic forces for Saint Venant-Kirchhoff material
// In : number of vertices, vertice positions, 
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				matrix FM1, FM2 and FM3 determined in initialisation
//				intial volume of tetrahedra
// Out : Force field on vertices (initialized in zero for input)
{
	int m_0, m_1, m_2, m_3;
	int i,j,k;

	double DeltatL[6];
	double V[9];
	double DeltaLFMi[12];

	CalculLongLiaisons(numLiaison,LinkExt,deltaLong2);

	for(i=0; i<numTetra; i++)
	{
		// 4 current tetrahedron vertices
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];
		// 6 lengths of edges
		for(j=0;j<6;j++)
            DeltatL[j]=deltaLong2[tetraLink[i][j]];
		
		// Vectors ve1-3
		for(j=0;j<3;j++)
		{
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		for(j=0;j<12;j++)
		{
			DeltaLFMi[j]  =0.;
		}

		// Calculate MF=V0*v*[DeltaL'*FM1;DeltaL'*FM2;DeltaL'*FM3]
		for(k=0;k<6;k++)
		{
			DeltaLFMi[0]+= DeltatL[k]* FM1[i][4*k];
			DeltaLFMi[1]+= DeltatL[k] *FM1[i][4*k+1];
			DeltaLFMi[2]+= DeltatL[k] *FM1[i][4*k+2];
			DeltaLFMi[3]+= DeltatL[k] *FM1[i][4*k+3];

			DeltaLFMi[4]+= DeltatL[k]* FM2[i][4*k];
			DeltaLFMi[5]+= DeltatL[k] *FM2[i][4*k+1];
			DeltaLFMi[6]+= DeltatL[k] *FM2[i][4*k+2];
			DeltaLFMi[7]+= DeltatL[k] *FM2[i][4*k+3];

			DeltaLFMi[8]+= DeltatL[k] *FM3[i][4*k];
			DeltaLFMi[9]+= DeltatL[k] *FM3[i][4*k+1];
			DeltaLFMi[10]+= DeltatL[k]*FM3[i][4*k+2];
			DeltaLFMi[11]+= DeltatL[k]*FM3[i][4*k+3];
		}

		// Normalization by the value of initial volume
		for(k=0;k<3;k++)
		{
            force[m_0][k]-=InitVolume[i]*(V[3*k]*DeltaLFMi[0] + V[3*k+1]*DeltaLFMi[4]	+V[3*k+2]*DeltaLFMi[8]);
			force[m_1][k]-=InitVolume[i]*(V[3*k]*DeltaLFMi[1] + V[3*k+1]*DeltaLFMi[5]	+V[3*k+2]*DeltaLFMi[9]);
			force[m_2][k]-=InitVolume[i]*(V[3*k]*DeltaLFMi[2] + V[3*k+1]*DeltaLFMi[6]	+V[3*k+2]*DeltaLFMi[10]);
			force[m_3][k]-=InitVolume[i]*(V[3*k]*DeltaLFMi[3] + V[3*k+1]*DeltaLFMi[7]	+V[3*k+2]*DeltaLFMi[11]);
		}
	}
	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculForcesHyperElastiques_NH(int numVertices, TabDblPtr *sommet, int numTetra, int **tetra, 
											double *InitVolume, double lambda0, double mu0, 
											TabDblPtr *FT1, TabDblPtr *FT2, TabDblPtr *FT3, double **force)
// Calculate hyperelastic forces for Néo-Hookéan material
// In : number of vertices, vertice positions,
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				intial volume of tetrahedra,
//				material parameters lambda0 and mu0,
//				matrix FT1, FT2 and FT3 determined in initialisation
// Out : Force field on vertices (initialized in zero for input)
{
	int m_0, m_1, m_2, m_3;
	int i,j,k;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	double Vy1, Vy2, Vy3;
	double Vz1, Vz2, Vz3;
	double SignedVolume,ConstVol;
	double J,logJ;

	double DeltatL[6];
	double V[9];
	double DeltaLFMi[12];

	CalculLongLiaisons(numLiaison,LinkExt,deltaLong2);

	for(i=0; i<numTetra; i++)
	{
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];

		X_i = sommet[m_0][0];
		Y_i = sommet[m_0][1] ;
		Z_i = sommet[m_0][2] ;  
		// ve1
		dx1 = sommet[m_1][0] - X_i ;  
		dy1 = sommet[m_1][1] - Y_i; 
		dz1 = sommet[m_1][2] - Z_i; 
		// ve2
		dx2 = sommet[m_2][0] - X_i ;  
		dy2 = sommet[m_2][1] - Y_i; 
		dz2 = sommet[m_2][2] - Z_i; 
		// ve3
		dx3 = sommet[m_3][0]- X_i ;  
		dy3 = sommet[m_3][1]- Y_i; 
		dz3 = sommet[m_3][2]- Z_i; 

		// v23 = ve2 x ve3
		Vx1 = dy2*dz3 - dz2*dy3; 
		Vy1 = dz2*dx3 - dx2*dz3;
		Vz1 = dx2*dy3 - dy2*dx3; 

		// v31 = ve3 x ve1
		Vx2 = dy3*dz1 - dz3*dy1; 
		Vy2 = dz3*dx1 - dx3*dz1; 
		Vz2 = dx3*dy1 - dy3*dx1; 

		// v12 = ve1 x ve2
		Vx3 = dy1*dz2 - dz1*dy2; 
		Vy3 = dz1*dx2 - dx1*dz2; 
		Vz3 = dx1*dy2 - dy1*dx2;

		SignedVolume = (dx1*Vx1 + dy1*Vy1 + dz1*Vz1)/6 ;

		VolumeGlobal+=SignedVolume;
		J = SignedVolume/InitVolume[i];

		static double seuilJ = 0.3;
		static double C_a =(lambda0-lambda0*log(seuilJ)-mu0)/(6*seuilJ*seuilJ);
		static double C_b =(lambda0-2*lambda0*log(seuilJ))/(6*seuilJ);
		
		// Calculation of volume forces
		if (J>seuilJ)
		{
			logJ = log(fabs(J));
			ConstVol = (lambda0*logJ-mu0)/(6*J);
		}
		else
		{
			ConstVol = C_a*J-C_b;
		}	

		force[m_1][0] -= ConstVol * Vx1; // v23
		force[m_1][1] -= ConstVol * Vy1; 
		force[m_1][2] -= ConstVol * Vz1; 

		force[m_2][0] -= ConstVol * Vx2; // v31
		force[m_2][1] -= ConstVol * Vy2; 
		force[m_2][2] -= ConstVol * Vz2; 

		force[m_3][0] -= ConstVol * Vx3; // v12
		force[m_3][1] -= ConstVol * Vy3; 
		force[m_3][2] -= ConstVol * Vz3; 

		force[m_0][0] += ConstVol * (Vx1 + Vx2 + Vx3); 
		force[m_0][1] += ConstVol * (Vy1 + Vy2 + Vy3); 
		force[m_0][2] += ConstVol * (Vz1 + Vz2 + Vz3);


		for(j=0;j<6;j++)
            DeltatL[j]=deltaLong2[tetraLink[i][j]];
		
		// Vectors ve1-3
		for(j=0;j<3;j++)
		{
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		for(j=0;j<12;j++)
		{
			DeltaLFMi[j]  =0.;
		}

		// Calculate MF=V0*v*[FT1;FT2;FT3]
		for(k=0;k<3;k++)
		{
            force[m_0][k]-=InitVolume[i]*(V[3*k]*FT1[i][0] + V[3*k+1]*FT2[i][0] + V[3*k+2]*FT3[i][0]);
            force[m_1][k]-=InitVolume[i]*(V[3*k]*FT1[i][1] + V[3*k+1]*FT2[i][1] + V[3*k+2]*FT3[i][1]);
            force[m_2][k]-=InitVolume[i]*(V[3*k]*FT1[i][2] + V[3*k+1]*FT2[i][2] + V[3*k+2]*FT3[i][2]);
            force[m_3][k]-=InitVolume[i]*(V[3*k]*FT1[i][3] + V[3*k+1]*FT2[i][3] + V[3*k+2]*FT3[i][3]);
		}
	}

	return OK;
}

int Maille::CalculForcesHyperElastiques_Mooney(TabDblPtr *sommet, int numTetra, int **tetra,double *InitVolume, double c01, double c10, 
											   TabDblPtr *FR1, TabDblPtr *FR2,TabDblPtr *FR3, TabDblPtr *FMR1, TabDblPtr *FMR2,TabDblPtr *FMR3, double **force)
// Calculate hyperelastic forces for Mooney-Rivlin material
// In : number of vertices, vertice positions,
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				intial volume of tetrahedra,
//				material parameters c01 and c10,
//				initialisatiaon matrices
// Out : Force field on vertices (initialized in zero for input)
{
	int m_0, m_1, m_2, m_3;
	int i,j,k;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	double Vy1, Vy2, Vy3;
	double Vz1, Vz2, Vz3;
	double SignedVolume,ConstVol;
	double VtrL;  //Vtr*L
	double J,logJ;

	double DeltatL[6];
	double L[6];
	double V[9];
	double DeltaLFMi[12];

	double dwdl[6];
	double LMtr[6]; //Lt*Mtr

	CalculLongLiaisons(numLiaison,LinkExt,deltaLong2);

	for(i=0; i<numTetra; i++)
	{
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];

		X_i = sommet[m_0][0];
		Y_i = sommet[m_0][1] ;
		Z_i = sommet[m_0][2] ;  
		// ve1
		dx1 = sommet[m_1][0] - X_i ;  
		dy1 = sommet[m_1][1] - Y_i; 
		dz1 = sommet[m_1][2] - Z_i; 
		// ve2
		dx2 = sommet[m_2][0] - X_i ;  
		dy2 = sommet[m_2][1] - Y_i; 
		dz2 = sommet[m_2][2] - Z_i; 
		// ve3
		dx3 = sommet[m_3][0]- X_i ;  
		dy3 = sommet[m_3][1]- Y_i; 
		dz3 = sommet[m_3][2]- Z_i; 

		// v23 = ve2 x ve3
		Vx1 = dy2*dz3 - dz2*dy3; 
		Vy1 = dz2*dx3 - dx2*dz3;
		Vz1 = dx2*dy3 - dy2*dx3; 

		// v31 = ve3 x ve1
		Vx2 = dy3*dz1 - dz3*dy1; 
		Vy2 = dz3*dx1 - dx3*dz1; 
		Vz2 = dx3*dy1 - dy3*dx1; 

		// v12 = ve1 x ve2
		Vx3 = dy1*dz2 - dz1*dy2; 
		Vy3 = dz1*dx2 - dx1*dz2; 
		Vz3 = dx1*dy2 - dy1*dx2;

		SignedVolume = (dx1*Vx1 + dy1*Vy1 + dz1*Vz1)/6 ;

		VolumeGlobal+=SignedVolume;
		J = SignedVolume/InitVolume[i];

		static double seuilJ = 0.3;

		static double C_a =(c10-c10*log(seuilJ)-2*c01)/(6*seuilJ*seuilJ);
		static double C_b =(c10-2*c10*log(seuilJ))/(6*seuilJ);
		
		if (J>seuilJ)
		{
			logJ = log(fabs(J));
			ConstVol = (c10*logJ-2*c01)/(6*J);
		}
		else
		{
			ConstVol = C_a*J-C_b;
		}	

		force[m_1][0] -= ConstVol * Vx1; // v23
		force[m_1][1] -= ConstVol * Vy1; 
		force[m_1][2] -= ConstVol * Vz1; 

		force[m_2][0] -= ConstVol * Vx2; // v31
		force[m_2][1] -= ConstVol * Vy2; 
		force[m_2][2] -= ConstVol * Vz2; 

		force[m_3][0] -= ConstVol * Vx3; // v12
		force[m_3][1] -= ConstVol * Vy3; 
		force[m_3][2] -= ConstVol * Vz3; 

		force[m_0][0] += ConstVol * (Vx1 + Vx2 + Vx3); 
		force[m_0][1] += ConstVol * (Vy1 + Vy2 + Vy3); 
		force[m_0][2] += ConstVol * (Vz1 + Vz2 + Vz3); 

		for(j=0;j<6;j++)
            DeltatL[j]=deltaLong2[tetraLink[i][j]]; 


		for(j=0;j<6;j++)
			L[j]=LinkLength2[tetraLink[i][j]];
		
		// Vectors ve1-3
		for(j=0;j<3;j++)
		{
			V[3*j]  =sommet[m_1][j]-sommet[m_0][j];
			V[3*j+1]=sommet[m_2][j]-sommet[m_0][j];
			V[3*j+2]=sommet[m_3][j]-sommet[m_0][j];
		}

		for(j=0;j<12;j++)
		{
			DeltaLFMi[j]  =0.;
		}

		for(k=0;k<6;k++)
		{
			DeltaLFMi[0]+= DeltatL[k]* FR1[i][4*k];
			DeltaLFMi[1]+= DeltatL[k] *FR1[i][4*k+1];
			DeltaLFMi[2]+= DeltatL[k] *FR1[i][4*k+2];
			DeltaLFMi[3]+= DeltatL[k] *FR1[i][4*k+3];

			DeltaLFMi[4]+= DeltatL[k]* FR2[i][4*k];
			DeltaLFMi[5]+= DeltatL[k] *FR2[i][4*k+1];
			DeltaLFMi[6]+= DeltatL[k] *FR2[i][4*k+2];
			DeltaLFMi[7]+= DeltatL[k] *FR2[i][4*k+3];

			DeltaLFMi[8]+= DeltatL[k] *FR3[i][4*k];
			DeltaLFMi[9]+= DeltatL[k] *FR3[i][4*k+1];
			DeltaLFMi[10]+= DeltatL[k]*FR3[i][4*k+2];
			DeltaLFMi[11]+= DeltatL[k]*FR3[i][4*k+3];
		}

		for(k=0;k<3;k++)
		{
            force[m_0][k]-=InitVolume[i]*(V[3*k]*FMR1[i][0]+V[3*k]*DeltaLFMi[0] + V[3*k+1]*FMR2[i][0]+V[3*k+1]*DeltaLFMi[4] + V[3*k+2]*FMR3[i][0]+V[3*k+2]*DeltaLFMi[8]);
            force[m_1][k]-=InitVolume[i]*(V[3*k]*FMR1[i][1]+V[3*k]*DeltaLFMi[1] + V[3*k+1]*FMR2[i][1]+V[3*k+1]*DeltaLFMi[5] + V[3*k+2]*FMR3[i][1]+V[3*k+2]*DeltaLFMi[9]);
            force[m_2][k]-=InitVolume[i]*(V[3*k]*FMR1[i][2]+V[3*k]*DeltaLFMi[2] + V[3*k+1]*FMR2[i][2]+V[3*k+1]*DeltaLFMi[6] + V[3*k+2]*FMR3[i][2]+V[3*k+2]*DeltaLFMi[10]);
            force[m_3][k]-=InitVolume[i]*(V[3*k]*FMR1[i][3]+V[3*k]*DeltaLFMi[3] + V[3*k+1]*FMR2[i][3]+V[3*k+1]*DeltaLFMi[7] + V[3*k+2]*FMR3[i][3]+V[3*k+2]*DeltaLFMi[11]);
		}
	}
		return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculAjoutForcesVolume_MS3D(TabDblPtr *sommet, int numTetra, int **tetra, 
							double *InitVolume, double raideurVolume, double **force)
// Calculation and add volume penalty forces to Mass-Spring 3D
// In :	vertice positions,
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				intial volume of tetrahedra,
//				volume stiffness parameter
// Out : Force field on vertices 
{
	int i;
	int   n_i, n_j, n_k, n_p;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	double Vy1, Vy2, Vy3;
	double Vz1, Vz2, Vz3;
	double SignedVolume,ConstVol,DeltaV;

	for (i=0; i<numTetra; i++)
	{
		n_i = tetra[i][0];
		n_j = tetra[i][1];
		n_k = tetra[i][2];
		n_p = tetra[i][3];

		X_i = sommet[n_i][0];
		Y_i = sommet[n_i][1] ;
		Z_i = sommet[n_i][2] ;  
		// ve1
		dx1 = sommet[n_j][0] - X_i ;  
		dy1 = sommet[n_j][1] - Y_i; 
		dz1 = sommet[n_j][2] - Z_i; 
		// ve2
		dx2 = sommet[n_k][0] - X_i ;  
		dy2 = sommet[n_k][1] - Y_i; 
		dz2 = sommet[n_k][2] - Z_i; 
		// ve3
		dx3 = sommet[n_p][0]- X_i ;  
		dy3 = sommet[n_p][1]- Y_i; 
		dz3 = sommet[n_p][2]- Z_i; 

		Vx1 = dy2*dz3 - dz2*dy3; 
		Vx2 = dz1*dy3 - dy1*dz3; 
		Vx3 = dy1*dz2 - dz1*dy2; 

		Vy1 = dz2*dx3 - dx2*dz3;
		Vy2 = dx1*dz3 - dz1*dx3; 
		Vy3 = dz1*dx2 - dx1*dz2; 

		Vz1 = dx2*dy3 - dy2*dx3;
		Vz2 = dy1*dx3 - dx1*dy3;
		Vz3 = dx1*dy2 - dy1*dx2;

		SignedVolume = (dx1*Vx1 + dx2*Vx2 + dx3*Vx3)/6 ;
		VolumeGlobal+=SignedVolume;

		DeltaV =(SignedVolume - InitVolume[i]);

		ConstVol = DeltaV * raideurVolume;

		force[n_j][0] -= ConstVol * Vx1; 
		force[n_j][1] -= ConstVol * Vy1; 
		force[n_j][2] -= ConstVol * Vz1; 

		force[n_k][0] -= ConstVol * Vx2; 
		force[n_k][1] -= ConstVol * Vy2; 
		force[n_k][2] -= ConstVol * Vz2; 

		force[n_p][0] -= ConstVol * Vx3; 
		force[n_p][1] -= ConstVol * Vy3; 
		force[n_p][2] -= ConstVol * Vz3; 

		force[n_i][0] += ConstVol * (Vx1 + Vx2 + Vx3); 
		force[n_i][1] += ConstVol * (Vy1 + Vy2 + Vy3); 
		force[n_i][2] += ConstVol * (Vz1 + Vz2 + Vz3);
	}

	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////


int Maille::CalculAjoutForcesVolume_NH(TabDblPtr *sommet, int numTetra, int **tetra, 
									   double *InitVolume, double lambda0, double mu0, double **force)
// Calculate and add volume penalty forces for compressible Neo-Hooke material
// In :	vertice positions, 
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				intial volume of tetrahedra,
//				material parameters lambda0 and mu0
// Out : Force field on vertices 
{
	int i;
	int   n_i, n_j, n_k, n_p;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	double Vy1, Vy2, Vy3;
	double Vz1, Vz2, Vz3;
	double SignedVolume,ConstVol;
	double J,logJ;

	for (i=0; i<numTetra; i++)
	{
		n_i = tetra[i][0];
		n_j = tetra[i][1];
		n_k = tetra[i][2];
		n_p = tetra[i][3];

		X_i = sommet[n_i][0];
		Y_i = sommet[n_i][1] ;
		Z_i = sommet[n_i][2] ;  
		// ve1
		dx1 = sommet[n_j][0] - X_i ;  
		dy1 = sommet[n_j][1] - Y_i; 
		dz1 = sommet[n_j][2] - Z_i; 
		// ve2
		dx2 = sommet[n_k][0] - X_i ;  
		dy2 = sommet[n_k][1] - Y_i; 
		dz2 = sommet[n_k][2] - Z_i; 
		// ve3
		dx3 = sommet[n_p][0]- X_i ;  
		dy3 = sommet[n_p][1]- Y_i; 
		dz3 = sommet[n_p][2]- Z_i; 

		Vx1 = dy2*dz3 - dz2*dy3; 
		Vx2 = dz1*dy3 - dy1*dz3; 
		Vx3 = dy1*dz2 - dz1*dy2; 

		Vy1 = dz2*dx3 - dx2*dz3;
		Vy2 = dx1*dz3 - dz1*dx3; 
		Vy3 = dz1*dx2 - dx1*dz2; 

		Vz1 = dx2*dy3 - dy2*dx3; 
		Vz2 = dy1*dx3 - dx1*dy3; 
		Vz3 = dx1*dy2 - dy1*dx2;

		SignedVolume = (dx1*Vx1 + dx2*Vx2 + dx3*Vx3)/6 ;
		VolumeGlobal+=fabs(SignedVolume);
		J = SignedVolume/InitVolume[i];

		static double seuilJ = 0.4;
		static double C_a =(lambda0-lambda0*log(seuilJ)-mu0)/(6*seuilJ*seuilJ);
		static double C_b =(lambda0-2*lambda0*log(seuilJ))/(6*seuilJ);
		
		if (J>seuilJ)
		{
			logJ = log(fabs(J));
			ConstVol = (lambda0*logJ-mu0)/(6*J);
		}
		else
		{
			ConstVol = C_a*J-C_b;
		}	

		force[n_j][0] -= ConstVol * Vx1; 
		force[n_j][1] -= ConstVol * Vy1; 
		force[n_j][2] -= ConstVol * Vz1; 

		force[n_k][0] -= ConstVol * Vx2; 
		force[n_k][1] -= ConstVol * Vy2; 
		force[n_k][2] -= ConstVol * Vz2; 

		force[n_p][0] -= ConstVol * Vx3; 
		force[n_p][1] -= ConstVol * Vy3; 
		force[n_p][2] -= ConstVol * Vz3; 

		force[n_i][0] += ConstVol * (Vx1 + Vx2 + Vx3); 
		force[n_i][1] += ConstVol * (Vy1 + Vy2 + Vy3); 
		force[n_i][2] += ConstVol * (Vz1 + Vz2 + Vz3);
	}

	return OK;
}


///////////////////////////////////////////////////////////////////////////////////////////


int Maille::CalculAjoutGravite(int numVertices, double **force)
// Calculate and add gravity forces applied to the vertices
// In : number of vertices,
// Out : Force field on vertices
{
	for(int i=0;i<numVertices;i++)
	{
		force[i][0] += gravite[0];
		force[i][1] += gravite[1];
		force[i][2] += gravite[2];
	}
	return OK;
}


///////////////////////////////////////////////////////////////////////////////////////////
int Maille::CalculAjoutForcesVisqueuses(double amortissement,
										int numVertices, TabDblPtr *sommet,  TabDblPtr *vitesse, 
										int numTetra, int **tetra, double *InitVolume, double **force)
// Calculate and add the force viscous
// In :	damping factor, number of vertices, vertice positions, velocities of vertices
//				number of tetrahedra and indices of tetrahedra vertex (tetra),
//				intial volume of tetrahedra,
// Out : Force field on vertices
{
	int m_0, m_1, m_2, m_3;

	for(int i=0; i<numTetra; i++)
	{
		m_0 = tetra[i][0];
		m_1 = tetra[i][1];
		m_2 = tetra[i][2];
		m_3 = tetra[i][3];

		double vitesseGravTetra[3]; 
		for(int k=0;k<3;k++)
		{
            vitesseGravTetra[k]=(vitesse[m_0][k]+vitesse[m_1][k]+vitesse[m_2][k]+vitesse[m_3][k])/4.;
			force[m_0][k]-=InitVolume[i]*amortissement*(vitesse[m_0][k]-vitesseGravTetra[k]);
			force[m_1][k]-=InitVolume[i]*amortissement*(vitesse[m_1][k]-vitesseGravTetra[k]); 
			force[m_2][k]-=InitVolume[i]*amortissement*(vitesse[m_2][k]-vitesseGravTetra[k]);
			force[m_3][k]-=InitVolume[i]*amortissement*(vitesse[m_3][k]-vitesseGravTetra[k]);
		}
	}
	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculAjoutForcesVisqueuses_MS3D(double *Raideur,
									int numVertices, double **vitesse, 
									int **LinkExt,int *TnumLiaisonAss, int **LiaisonAss,
									double **force)
// Calculate and add viscous forces according to links for mass-spring 3D
// In :	number of vertices,  
//				velocities of vertices,
//				number of links, indices of links,
//				stiffness table
//				number of links associated with vertices and tables of related links 
// Out : Force field on vertices (added values)
{
	int i,j,k;

	for(i=0;i<numVertices;i++)
	{
		for(j=0;j<TnumLiaisonAss[i];j++)
        {
			k=LiaisonAss[i][j]; // number of considered links

			if(i==LinkExt[k][0]) // Index of the first end of the link
			{
				force[i][0]-=Raideur[k]*(vitesse[i][0]-vitesse[LinkExt[k][1]][0]);
				force[i][1]-=Raideur[k]*(vitesse[i][1]-vitesse[LinkExt[k][1]][1]);
				force[i][2]-=Raideur[k]*(vitesse[i][2]-vitesse[LinkExt[k][1]][2]);
			}
			else
			{
				force[i][0]-=Raideur[k]*(vitesse[i][0]-vitesse[LinkExt[k][0]][0]);
				force[i][1]-=Raideur[k]*(vitesse[i][1]-vitesse[LinkExt[k][0]][1]);
				force[i][2]-=Raideur[k]*(vitesse[i][2]-vitesse[LinkExt[k][0]][2]);
			}
	    }
	}
	return OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int Maille::CalculForces_et_Integration()
{
	// Startup timer
	static int itime=0;
	static int ncount = 1000;
	static __int64 startTime;
	if(itime==0) oPreciseTimer.StartTimer();
	int i;

	// Attach the interactive position of the sphere with a vertex of the mesh
	if(firstCapture)
	{
		if(fix==0)
		{
			sommet[0][0]=xSphere;       
			sommet[0][1]=ySphere;     
			sommet[0][2]=zSphere;     
		}
		else 
		{
			sommet[100][0]=xSphere;       
			sommet[100][1]=ySphere;     
			sommet[100][2]=zSphere;     
		}
	}

	for(i=0; i<numVertices; i++)
	{
		force[i][0]=0.;  
		force[i][1]=0.;
		force[i][2]=0.;

		gama[i][0]=0.;
		gama[i][1]=0.;
		gama[i][2]=0.;
	}

	VolumeGlobal=0.;

	// Calculate the forces hyperelastique
	switch(MATERIAL_TYPE)
    {
	case MS3D: // Mass-Spring 3D
		CalculForcesElastiques_MS3D(numVertices, sommet, numLiaison, LinkExt,
												TnumLiaisonAss, LiaisonAss, force);
		break;
	case STVK: // Saint Venant-Kirchhoff
		CalculForcesHyperElastiques_STVK(numVertices,sommet,numTetra,
										tetra,InitVolume,FM1,FM2,FM3,force);
		break;
	case NH: // Néo-Hookéen
		CalculForcesHyperElastiques_NH(numVertices,sommet,numTetra,
										tetra,InitVolume,lambda0,mu0,FT1,FT2,FT3,force);
		break;
	case Mooney: // Mooney-Rivlin
		CalculForcesHyperElastiques_Mooney(sommet,numTetra,tetra,InitVolume,c01,c10,FR1, FR2,FR3, FMR1, FMR2,FMR3,force);
		break;
    }

	//////////Visco-elastique force Prony series 
	double alphai=0.5;      
	double taoi=0.58;
	double ai, bi;
	
	ai=dt*alphai/(dt+taoi);
	bi=taoi/(dt+taoi);

	for(i=0; i<numVertices; i++)
	{
		gama[i][0]=ai*force[i][0]+bi*gama[i][0];
		gama[i][1]=ai*force[i][1]+bi*gama[i][1];
		gama[i][2]=ai*force[i][2]+bi*gama[i][2];
	}

	for(i=0; i<numVertices; i++)
	{
		forcevisco[i][0]=force[i][0]-gama[i][0];
		forcevisco[i][1]=force[i][1]-gama[i][1];
		forcevisco[i][2]=force[i][2]-gama[i][2];
	}

	for(i=0; i<numVertices; i++)
	{
		force[i][0]=force[i][0]+forcevisco[i][0];
		force[i][1]=force[i][1]+forcevisco[i][1];
		force[i][2]=force[i][2]+forcevisco[i][2];
	}
	//////end visco

	// Add gravity (volume force)
	CalculAjoutGravite(numVertices,force);

	// Add force viscous 
	if ((MATERIAL_TYPE==STVK) || (MATERIAL_TYPE==NH))

		CalculAjoutForcesVisqueuses(amortissement,numVertices,sommet,vitesse,numTetra,tetra,InitVolume,force);
		
	if (MATERIAL_TYPE==MS3D)
		CalculAjoutForcesVisqueuses_MS3D(Raideur,numVertices,vitesse,LinkExt,
												TnumLiaisonAss,LiaisonAss,force);

	// Calculation of velocities and positions by Euler semi-implicit integrating
	for(i=0;i<numVertices;i++) 
	{
		// Static constraint
		if (fix!=0) {if (i<fix) continue;}

		// Integration of velocities 
		vitesse[i][0]+=dt*force[i][0]/masse[i];
		vitesse[i][1]+=dt*force[i][1]/masse[i];
		vitesse[i][2]+=dt*force[i][2]/masse[i];

		// Integration of positions
		sommet[i][0]+=dt*vitesse[i][0];
		sommet[i][1]+=dt*vitesse[i][1];
		sommet[i][2]+=dt*vitesse[i][2];

		// Constraints of non penetration barrier plane
		if(!Indent)
		{  
			if (sommet[i][1]<yplan) // barrier plane
			{
				sommet[i][1]=yplan;
				float Krebond=0.6;
				vitesse[i][1]=-Krebond*vitesse[i][1];
			}
		}

		// Constraints of indentation - Compression - extension
		if(Indent)
		{
			//if(i<25) // cube_xav_5
			if(i<4) // cube_unique
			// if(i>0) // tetra
			{
				vitesse[i][0]=0.;
				sommet[i][0]=yIndent_inf;
			}

			//if(i>=100) // cube_xav_5 
			if(i>=4) // cube_unique
			// if(i==0) // tetra
			{
				vitesse[i][0]=0.;
				sommet[i][0]=yIndent_sup;
			}
		}
	}


	// Indentation - compression/extension
	
	//static double Vind=-0.5f; // cube_xav_5
	static double Vind=0.001f;
	//static double Vind=-0.2f;
	// static double Vind=-0.01f; // tetra
	static int compteur_ind;
	static int Compteur_max=20000000;
	static int modulo=1000; // cube_xav_5
	// static int modulo=20000; // cube_unique
	// static int modulo=200000; // tetra

	if(Indent)
	{
		if(!(compteur_ind%modulo)) // Registration and change all the "modulo" times
			{
				// Calculate the resultant force of the upper side (for indentation)
				force_indent=0.;
				//for (int i=100;i<125;i++)	force_indent+=force[i][2]; // cube_xav_5
				//for (int i=4;i<8;i++) force_indent+=force[i][2]; // cube_unique
				force_indent+=force[5][0]; // tetra
				//force_indent+=force[112][2]; // cube_xav_5
				fprintf(pfile_acq,"%e %e %e\n",yIndent_sup,force_indent,VolumeGlobal);
				yIndent_sup+=Vind*modulo*dt;
			}
		compteur_ind++;	
	}

	// Startup the indentation
	if(StartIndent)
	{
		//yIndent_sup=(sommet[4][2]+sommet[5][2]+sommet[6][2]+sommet[7][2])/4.; // cube_unique
		//yIndent_sup=25.; // cube_xav_5
		// yIndent_sup=1.1; // tetra
		yIndent_sup=0.2; // tetra
		//yIndent_inf=-25.; // cube_unique et cube_xav_5
		 yIndent_inf=0.;

		compteur_ind=1;

		CString filename_acq;
		filename_acq=FilePath+".acq";
		if( (pfile_acq  = fopen( filename_acq, "w" )) == NULL )
		{
            printf( "\n-> ERROR: cannot open file '%s' !\n", filename_acq);
		}
		else
			printf( "\n-> The file '%s' was opened.\n", filename_acq );

		Indent = true;
		StartIndent = false;
	}

	// End of the indentation
	if((Indent)&&(compteur_ind>=Compteur_max))
	{
		fclose(pfile_acq);
		MessageBox(NULL,"Indentation finished","Indentation",0);
		Indent = false;			
	}
	if((Indent)&&(StopIndent))
	{
		fclose(pfile_acq);
		MessageBox(NULL,"Indentation interrupted","Indentation",0);
		Indent = false;
	}

		// Simulation / enregistrement des positions des noeuds
	
/*	static int compteur_simu;
	static int Compteur_simu_max=10000000;
	static int n_simu=4; // noeud no. n_simu 
	static int modul=10000; */

		//Contrainte pour modele foetus
	/*if(Simu)
	{
		sommet[37][0]=-17.962763;	sommet[37][1]=8.054344;	sommet[37][2]=17.990398;
		sommet[141][0]=-19.562756;	sommet[141][1]=15.569409;	sommet[141][2]=56.683643;
		sommet[161][0]=-2.36915;	sommet[161][1]=5.748552;	sommet[161][2]=18.30228;
		sommet[162][0]=-8.551289;	sommet[162][1]=8.657516;	sommet[162][2]=30.738096;
		sommet[190][0]=-21.413261;	sommet[190][1]=11.183702;	sommet[190][2]=31.287798;
		sommet[279][0]=-23.477037;	sommet[279][1]=13.769632;	sommet[279][2]=43.822701;
		sommet[280][0]=-12.248973;	sommet[280][1]=11.706466;	sommet[280][2]=44.088497;
		sommet[412][0]=-4.466551;	sommet[412][1]=13.0105;		sommet[412][2]=56.306858;
		sommet[633][0]=4.353496;	sommet[633][1]=8.087834;	sommet[633][2]=29.968365;
		sommet[594][0]=0.111013;	sommet[594][1]=10.328369;	sommet[594][2]=43.656513;

		sommet[392][2]=ztire;  //test foetus inden

		sommet[0][0]=0; sommet[0][1]=0; sommet[0][2]=0; 
		sommet[1][0]=0; sommet[1][1]=0; sommet[1][2]=0.2; 
		sommet[2][0]=0; sommet[2][1]=0.2; sommet[2][2]=0; 
		sommet[3][0]=0; sommet[3][1]=0.2; sommet[3][2]=0.2;     //test TestCube
	}*/

		/*sommet[4][0]=ztire;
		sommet[5][0]=ztire;
		sommet[6][0]=ztire;
		sommet[7][0]=ztire;
		ztire-=0.00000001;
	}

	
	if(Simu)
	{
		if(!(compteur_simu%modul)) fprintf(pfile_acq,"%e %e %e\n",sommet[n_simu][0],sommet[n_simu][1],sommet[n_simu][2]);
		compteur_simu++;	
	}

	// Démarrage de la simulation
	if(StartSimu)
	{
		compteur_simu=1;

		//ztire=191.623795; //test foetus inden

		ztire=0.2;

		// Ouverture fichier d'enregistrement des mesures
		CString filename_acq;
		filename_acq=FilePath+".simu";
		if( (pfile_acq  = fopen( filename_acq, "w" )) == NULL )
		{
            printf( "\n-> ERROR: cannot open file '%s' !\n", filename_acq);
		}
		else
			printf( "\n-> The file '%s' was opened.\n", filename_acq );

		Simu = true;
		StartSimu = false;
	}

	// Fins de la simulation
	if((Simu)&&(compteur_simu>=Compteur_simu_max))
	{
		fclose(pfile_acq);
		MessageBox(NULL,"Simulation terminée","Indentation",0);
		Simu = false;			
	}
	if((Simu)&&(StopSimu))
	{
		fclose(pfile_acq);
		MessageBox(NULL,"Simulation interrompue","Indentation",0);
		Simu = false;
	}*/


	// Timer
	itime++;
	if(itime==ncount)
	{
		oPreciseTimer.StopTimer();
		i64PreciseTimer = oPreciseTimer.GetTime()/ncount;
		itime=0;
	}
	return OK;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////

void Maille::charger_Masse()
{
	CString filename;
	filename=FilePath+".masse";
	FILE *pfile;
	int i;	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	masse= new double[numVertices];
	for(i=0; i<numVertices;i++)
	{
		fscanf(pfile,"%lf",&(masse[i])); //here masse[i] is the coef of mass
		masse[i]=masse[i]*masseVolumique; // now masse[i] is the mass

		fprintf(pfile_acq,"Masses n %i : %e\n",i,masse[i]);
	}
	fclose(pfile);
}

void Maille::Calcul_Masse()
{
	CString filename;
	filename=FilePath+".Volum";
	FILE *pfile;
	int i;	
	if( (pfile  = fopen( filename, "r" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	double *Coef_masse= new double[numVertices];
	for(i=0; i<numVertices;i++)
		Coef_masse[i]=0.;
	
	double *Volume=new double[numTetra];
	for(i=0; i<numTetra;i++)
	{
		fscanf(pfile,"%lf",&(Volume[i]));
		
		Coef_masse[tetra[i][0]]+=Volume[i]/4.;
		Coef_masse[tetra[i][1]]+=Volume[i]/4.;
		Coef_masse[tetra[i][2]]+=Volume[i]/4.;
		Coef_masse[tetra[i][3]]+=Volume[i]/4.;
	}
	
	CString filename1;
	filename1=FilePath+".masse";
	FILE *pfile1;
	
	if( (pfile1  = fopen( filename1, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename1);
	else
		printf( "\n-> The file '%s' was opened.\n", filename1 );
	
	for(i=0; i<numVertices;i++)
		fprintf(pfile1,"%lf\n",Coef_masse[i]);
	
	fclose(pfile);
	fclose(pfile1);
	delete(Volume);
}

void Maille::Calcul_Volume()
{
	CString filename;
	filename=FilePath+".Volum";
	FILE *pfile;
	
	if( (pfile  = fopen( filename, "w" )) == NULL )
		printf( "\n-> ERROR: cannot open file '%s' !\n", filename);
	else
		printf( "\n-> The file '%s' was opened.\n", filename );
	
	int   n_i, n_j, n_k, n_p;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	double Vy1, Vy2, Vy3;
	double Vz1, Vz2, Vz3;
	double 	Volume_temp;
		
	for(int i=0;i<numTetra;i++)
	{
		n_i = tetra[i][0];
		n_j = tetra[i][1];
		n_k = tetra[i][2];
		n_p = tetra[i][3];

		X_i  =  sommet[n_i][0];
		Y_i  =  sommet[n_i][1] ;
		Z_i  =  sommet[n_i][2] ;  

		dx1 = sommet[n_j][0] - X_i ;  
		dy1 = sommet[n_j][1] - Y_i; 
		dz1 = sommet[n_j][2] - Z_i; 

		dx2 = sommet[n_k][0] - X_i ;  
		dy2 = sommet[n_k][1] - Y_i; 
		dz2 = sommet[n_k][2] - Z_i; 

		dx3 = sommet[n_p][0]- X_i ;  
		dy3 = sommet[n_p][1]- Y_i; 
		dz3 = sommet[n_p][2]- Z_i; 

		Vx1 = dy2*dz3 - dz2*dy3; 
		Vx2 = dz1*dy3 - dy1*dz3; 
		Vx3 = dy1*dz2 - dz1*dy2; 

		Vy1 = dz2*dx3 - dx2*dz3; 
		Vy2 = dx1*dz3 - dz1*dx3; 
		Vy3 = dz1*dx2 - dx1*dz2; 

		Vz1 = dx2*dy3 - dy2*dx3; 
		Vz2 = dy1*dx3 - dx1*dy3; 
		Vz3 = dx1*dy2 - dy1*dx2;

		Volume_temp = fabs(dx1*Vx1 + dx2*Vx2 + dx3*Vx3)/6. ; 
		fprintf(pfile,"%lf\n",Volume_temp);
	}
	fclose(pfile);	
}

void Maille::charger_Volume()
{
	int   n_i, n_j, n_k, n_p;
	double X_i, Y_i, Z_i;
	double dx1, dy1, dz1;
	double dx2, dy2, dz2;
	double dx3, dy3, dz3;
	double Vx1, Vx2, Vx3;
	
	for(int i=0;i<numTetra;i++)
	{
		n_i = tetra[i][0];
		n_j = tetra[i][1];
		n_k = tetra[i][2];
		n_p = tetra[i][3];

		X_i  =  sommet[n_i][0];
		Y_i  =  sommet[n_i][1] ;
		Z_i  =  sommet[n_i][2] ;  

		dx1 = sommet[n_j][0] - X_i ;  
		dy1 = sommet[n_j][1] - Y_i; 
		dz1 = sommet[n_j][2] - Z_i; 

		dx2 = sommet[n_k][0] - X_i ;  
		dy2 = sommet[n_k][1] - Y_i; 
		dz2 = sommet[n_k][2] - Z_i; 

		dx3 = sommet[n_p][0]- X_i ;  
		dy3 = sommet[n_p][1]- Y_i; 
		dz3 = sommet[n_p][2]- Z_i; 

		Vx1 = dy2*dz3 - dz2*dy3; 
		Vx2 = dz1*dy3 - dy1*dz3; 
		Vx3 = dy1*dz2 - dz1*dy2; 

		InitVolume[i] = dx1*Vx1 + dx2*Vx2 + dx3*Vx3 ;
		if (InitVolume[i]<0) 
		{
			tetra[i][1]=n_k;
			tetra[i][2]=n_j;
		}
		InitVolume[i]=fabs(InitVolume[i])/6.;
		cteVolume[i]=4*raideurVolume*dt*dt/(9*InitVolume[i]*InitVolume[i]);
	}
}

void Maille::Calcul_Raideur()
{
}


void Maille::drawString(CString string, GLfloat x, GLfloat y)
{ glRasterPos3f(x, y,-10.);
int i=0;  
while (string[i])
  { 
//    i++;
  }
}


//PreciseTimer
bool CPreciseTimer::sm_bInit = false;
bool CPreciseTimer::sm_bPerformanceCounter;
__int64 CPreciseTimer::sm_i64Freq;

//CONSTRUCTOR
CPreciseTimer::CPreciseTimer() : m_i64Start(0), m_i64Elapsed(0), m_bRunning(false)
{
	//Only if not already initialized
	if(false == sm_bInit)
	{
		//Initializing some static variables dependent on the system just once
		LARGE_INTEGER liFreq;
		if(TRUE == QueryPerformanceFrequency(&liFreq))
		{
			//Only if the system is supporting High Performance
			sm_i64Freq = ((__int64)liFreq.HighPart << 32) + (__int64)liFreq.LowPart;
			sm_bPerformanceCounter = true;
		}
		else
			sm_bPerformanceCounter = false;
		sm_bInit = true;
	}
}

void CPreciseTimer::StartTimer()
{
	if(true == sm_bPerformanceCounter)
	{
		QueryPerformanceCounter(&m_liCount);
		m_i64Start = ((__int64)m_liCount.HighPart << 32) + (__int64)m_liCount.LowPart;
		//Transform in microseconds
		(m_i64Start *= 1000000) /= sm_i64Freq;
	}
	else
		//Transform milliseconds to microseconds
		m_i64Start = (__int64)GetTickCount() * 1000;
	m_bRunning = true;
}

void CPreciseTimer::StopTimer()
{
	UpdateElapsed();
	m_bRunning = false;
}

__int64 CPreciseTimer::GetTime()
{
	if(true == m_bRunning)
		UpdateElapsed();
	return m_i64Elapsed;
}