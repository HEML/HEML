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


/////////////////////////////////////////////////////
//This file is the post-processer for visualisation//
//of simulation results using the HEML method      //
/////////////////////////////////////////////////////


#include "StdAfx.h"
#include "math.h"
#include <time.h>

#include "View_HEML.h"
#include "HEML.h"

Maille *Rein;
bool booltestCol;
int test_counter;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CRenderView

IMPLEMENT_DYNCREATE(CRenderView, CView)

BEGIN_MESSAGE_MAP(CRenderView, CView)
//{{AFX_MSG_MAP(CRenderView)
ON_WM_DESTROY()
ON_WM_SIZE()
ON_WM_LBUTTONDOWN()
ON_WM_LBUTTONUP()
ON_WM_MOUSEMOVE()
ON_WM_PAINT()
ON_WM_TIMER()
ON_WM_CREATE()
ON_WM_KEYDOWN()
ON_WM_RBUTTONDOWN()
ON_WM_RBUTTONUP()
//}}AFX_MSG_MAP
// Standard printing commands
ON_COMMAND(ID_FILE_PRINT, CView::OnFilePrint)
ON_COMMAND(ID_FILE_PRINT_DIRECT, CView::OnFilePrint)
ON_COMMAND(ID_FILE_PRINT_PREVIEW, CView::OnFilePrintPreview)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRenderView construction/destruction

CRenderView::CRenderView()
{
	// OpenGL
	m_hGLContext = NULL;
	m_GLPixelIndex = 0;
	
	// Mouse
	m_LeftButtonDown = FALSE;
	m_RightButtonDown = FALSE;
	m_SphereMove=FALSE;
	m_CursorRotation = AfxGetApp()->LoadCursor(IDC_CURSOR_ROTATION);

	// Colors
	CToolApp *pApp = (CToolApp *)AfxGetApp();
	m_ClearColorRed   = GetRValue(pApp->m_OptionColorGlBack);
	m_ClearColorGreen = GetGValue(pApp->m_OptionColorGlBack);
	m_ClearColorBlue  = GetBValue(pApp->m_OptionColorGlBack);


	Rein=new Maille;
	booltestCol=false;
	InitGeometry();
}

//********************************************
// InitGeometry
//********************************************
void CRenderView::InitGeometry(void)
{
	m_xRotation = 0.0f;
	m_yRotation = 0.0f;
	m_zRotation = 0.0f;

	m_xTranslation = 0.0f;
	m_yTranslation = 0.0f;
	m_zTranslation = 0.0f;

//	m_zoomslidefactor = (float)0.0001;
	m_zoomslidefactor = (float)0.0004;
//	m_zoomslidefactor = (float)0.001; // tetra
	m_slideinit = 10;
	m_xScaling = (float)m_slideinit*m_zoomslidefactor;
}


CRenderView::~CRenderView()
{
}

BOOL CRenderView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	
	return CView::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// CRenderView drawing

void CRenderView::OnDraw(CDC* pDC)
{
	CToolDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	
	// TODO: add draw code for native data here
}

/////////////////////////////////////////////////////////////////////////////
// CRenderView printing

BOOL CRenderView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CRenderView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CRenderView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}

/////////////////////////////////////////////////////////////////////////////
// CRenderView diagnostics

#ifdef _DEBUG
void CRenderView::AssertValid() const
{
	CView::AssertValid();
}

void CRenderView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CToolDoc* CRenderView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CToolDoc)));
	return (CToolDoc*)m_pDocument;
}
#endif //_DEBUG


//********************************************
// GetFormView
//********************************************
CFormCommandView* CRenderView::GetFormCommandView() 
{
	CToolApp *pApp = (CToolApp *)AfxGetApp();
	CMainFrame *pMainFrame = (CMainFrame *)pApp->m_pMainWnd;
	CChildFrame *pFrame = (CChildFrame *)pMainFrame->GetActiveFrame();
	
	CFormCommandView *pView = (CFormCommandView *)pFrame->m_wndSplitter.GetPane(0,0);
	return pView;
}


//////////////////////////////////////////////
//////////////////////////////////////////////
// OPENGL
//////////////////////////////////////////////
//////////////////////////////////////////////

//********************************************
// OnCreate
// Create OpenGL rendering context 
//********************************************
int CRenderView::OnCreate(LPCREATESTRUCT lpCreateStruct) 
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	HWND hWnd = GetSafeHwnd();
	HDC hDC = ::GetDC(hWnd);
	
	if(SetWindowPixelFormat(hDC)==FALSE)
		return 0;
	
	if(CreateViewGLContext(hDC)==FALSE)
		return 0;


	// Default mode
//	glPolygonMode(GL_FRONT,GL_LINE);
//	glPolygonMode(GL_BACK,GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glShadeModel(GL_FLAT);
	glEnable(GL_NORMALIZE);


	// Lights, material properties
  GLfloat	ambientProperties[]  = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat	diffuseProperties[]  = {0.8f, 0.8f, 0.8f, 1.0f};
  GLfloat	specularProperties[] = {1.0f, 1.0f, 1.0f, 1.0f};
	
  glClearDepth( 1.0 );
	
  glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties);
  glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
  glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

	// Default : lighting
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHTING);
glShadeModel(GL_SMOOTH);
//	pFormView = GetFormCommandView();
	
	return 0;
}

//********************************************
// SetWindowPixelFormat
//********************************************
BOOL CRenderView::SetWindowPixelFormat(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pixelDesc;
	
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;
	
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE;
	
	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 8;
	pixelDesc.cRedShift = 16;
	pixelDesc.cGreenBits = 8;
	pixelDesc.cGreenShift = 8;
	pixelDesc.cBlueBits = 8;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 64;
	pixelDesc.cAccumRedBits = 16;
	pixelDesc.cAccumGreenBits = 16;
	pixelDesc.cAccumBlueBits = 16;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 32;
	pixelDesc.cStencilBits = 8;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = PFD_MAIN_PLANE;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;
	
	m_GLPixelIndex = ChoosePixelFormat(hDC,&pixelDesc);
	if(m_GLPixelIndex == 0) // Choose default
	{
		m_GLPixelIndex = 1;
		if(DescribePixelFormat(hDC,m_GLPixelIndex,
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0)
			return FALSE;
	}
	
	if(!SetPixelFormat(hDC,m_GLPixelIndex,&pixelDesc))
		return FALSE;
	
	return TRUE;
}

//********************************************
// CreateViewGLContext
// Create an OpenGL rendering context
//********************************************
BOOL CRenderView::CreateViewGLContext(HDC hDC)
{
	m_hGLContext = wglCreateContext(hDC);
	
	if(m_hGLContext==NULL)
		return FALSE;
	
	if(wglMakeCurrent(hDC,m_hGLContext)==FALSE)
		return FALSE;
	
	return TRUE;
}

//********************************************
// OnDestroy
// Cleanup every OpenGL rendering context
//********************************************
void CRenderView::OnDestroy() 
{
	if(wglGetCurrentContext() != NULL)
		wglMakeCurrent(NULL,NULL);
	
	if(m_hGLContext != NULL)
	{
		wglDeleteContext(m_hGLContext);
		m_hGLContext = NULL;
	}
	Rein->~Maille();
	
	CView::OnDestroy();
}

//********************************************
// OnSize
//********************************************
void CRenderView::OnSize(UINT nType, int cx, int cy) 
{
	CView::OnSize(nType, cx, cy);
	
	// Set OpenGL perspective, viewport and mode
	CSize size(cx,cy);
	double aspect;
	aspect = (cy == 0) ? (double)size.cx : (double)size.cx/(double)size.cy;
	
	glViewport(0,0,size.cx,size.cy);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-2,2,-2,2,-100,100);
	//gluPerspective(45,aspect,1,15.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDrawBuffer(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	
}

//////////////////////////////////////////////
//////////////////////////////////////////////
// MOUSE
//////////////////////////////////////////////
//////////////////////////////////////////////
//********************************************
// Right button Mouse 
//********************************************
void CRenderView::OnRButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_RightButtonDown = TRUE;
	m_RightDownPos = point;
	CView::OnRButtonDown(nFlags, point);
}

void CRenderView::OnRButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_RightButtonDown = FALSE;
	CView::OnRButtonUp(nFlags, point);
}

//********************************************
// Left button Mouse 
//********************************************
void CRenderView::OnLButtonDown(UINT nFlags, 
																CPoint point) 
{
	m_LeftButtonDown = TRUE;
	m_LeftDownPos = point;
	CView::OnLButtonDown(nFlags, point);
}

void CRenderView::OnLButtonUp(UINT nFlags, 
															CPoint point) 
{

	m_LeftButtonDown = FALSE;
	CView::OnLButtonUp(nFlags, point);
}

void CRenderView::OnMouseMove(UINT nFlags, 
															CPoint point) 
{
	if(m_LeftButtonDown)
	{
		if(!m_SphereMove)
		{
			float Rotation_scale = 10.0f;  
			m_yRotation -= (float)(m_LeftDownPos.x - point.x)/Rotation_scale;
			m_xRotation -= (float)(m_LeftDownPos.y - point.y)/Rotation_scale;
			m_LeftDownPos = point;
			InvalidateRect(NULL,FALSE);
		}
		else
		{
			Rein->Curs_Pos[0]-=(float)(m_LeftDownPos.x - point.x)/10.0f; //6.0f
			Rein->Curs_Pos[1]+=(float)(m_LeftDownPos.y - point.y)/10.0f;
			m_LeftDownPos = point;
			InvalidateRect(NULL,FALSE);
		}
	}
	if(m_RightButtonDown)
	{
		if(!m_SphereMove)
		{
			m_zRotation -= (float)(m_RightDownPos.x - point.x)/30.0f;
			m_RightDownPos = point;
			InvalidateRect(NULL,FALSE);

		}
		else
		{
			Rein->Curs_Pos[2] -= (float)(m_RightDownPos.x - point.x)/30.0f;
			m_RightDownPos = point;
			InvalidateRect(NULL,FALSE);
		}
	}
	CView::OnMouseMove(nFlags, point);
}


//////////////////////////////////////////////
//////////////////////////////////////////////
// PAINTING 
//////////////////////////////////////////////
//////////////////////////////////////////////

//********************************************
// OnPaint
//********************************************
void CRenderView::OnPaint() 
{

	static BOOL FIRST_TIME = TRUE;

	if (FIRST_TIME) // Executé une seule fois, au début
	{
		// last_time = clock();
		pFormCommandView = GetFormCommandView(); 
		pFormCommandView->m_time4 = (double)(Rein->dt * 1000);
		FIRST_TIME = FALSE;
	}


	// Device context for painting
	CPaintDC dc(this); 
	
	// Model is stored in Document
	CToolDoc *pDoc = (CToolDoc *)GetDocument();
	ASSERT_VALID(pDoc);
	
	// Useful in multidoc templates
	HWND hWnd = GetSafeHwnd();
	HDC hDC = ::GetDC(hWnd);
	wglMakeCurrent(hDC,m_hGLContext);
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glClearColor(m_ClearColorRed,m_ClearColorGreen,m_ClearColorBlue,1.0f);
	
	glPushMatrix();
	
	// Position / translation / scale
	glTranslated(m_xTranslation,m_yTranslation,m_zTranslation);
	glRotatef(m_xRotation, 1.0, 0.0, 0.0);
	glRotatef(m_yRotation, 0.0, 1.0, 0.0);
	glRotatef(m_zRotation, 0.0, 0.0, 1.0);

//	glScalef(m_xScaling,m_yScaling,m_zScaling);
	glScalef(m_xScaling,m_xScaling,m_xScaling);
	
	// Start rendering...
//Rein->collision(sphere_x,sphere_y,sphere_z);	
//Rein->calcul();

/*	glPushMatrix();
	GLUquadricObj *quadObj;
    quadObj = gluNewQuadric ();
//    displayProbe = glGenLists(1);
//    glNewList(displaySettings1->preDisplayList, GL_COMPILE);
		//glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		//glTranslatef(0,0,0);
		//gluSphere(quadObj, 1,5,5);
		//gluCylinder(quadObj, 1,5,70,5,5);
		glColor3f( 0.5f, 0.5f, 0.0f );
		glTranslated(Rein->Curs_Pos[0],Rein->Curs_Pos[1],Rein->Curs_Pos[2]);
		gluSphere(quadObj, 2.,10,10);
//    glEndList();
//    gluDeleteQuadric(quadObj);
	
	glPopMatrix();
*/
	InvalidateRect(NULL,FALSE);



//	glPushMatrix();

//  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

	Rein->dessine_maille(); 

		pFormCommandView = GetFormCommandView(); 

//		pFormCommandView->m_time1 = (double)Rein->i64PreciseTimer;
		pFormCommandView->m_time1 = (float)(Rein->i64PreciseTimer)/1000;
//		pFormCommandView->m_time2 = 0;
//		pFormCommandView->m_time3 = 1;
		pFormCommandView->UpdateData(FALSE);

//	CString tempString="ddddddd";
//	Rein->drawString(tempString,50,50);
//	glPopMatrix();
	
/*	//pDoc->RenderScene();
	static int count = 0, int count_max = 10;
	static double somme_temps_calcul = 0;
	static double somme_temps_dessin = 0;
	static BOOL FIRST_TIME = TRUE;
	double somme_temps_total;
	static clock_t last_time, current_time;
	
	if (FIRST_TIME) // Executé une seule fois, au début
	{
		last_time = clock();
		pFormCommandView = GetFormCommandView(); 
		pFormCommandView->m_time4 = (double)(Rein->dt * 1000);
		FIRST_TIME = FALSE;
	}

	if (count < count_max)
	{
		count++;
		somme_temps_calcul += Rein->temps_calcul;
		somme_temps_dessin += Rein->temps_dessin;
	}
	else // Pas de test sur FIRST_TIME car inutile
	{
		current_time = clock();
		somme_temps_total = (double)(current_time - last_time) * 1000 / CLOCKS_PER_SEC;	
		pFormCommandView->m_time1 = somme_temps_calcul/count_max;
		pFormCommandView->m_time2 = somme_temps_dessin/count_max;
		pFormCommandView->m_time3 = somme_temps_total/count_max;
		pFormCommandView->UpdateData(FALSE);

		count = 0;
		somme_temps_calcul = 0;
		somme_temps_dessin = 0;
		last_time = current_time;
	}
*/

	glPopMatrix();
	
	// Double buffer
	SwapBuffers(dc.m_ps.hdc);
//Sleep(20);
}

//********************************************
// OnTimer
//********************************************
// Currently animation timer
//********************************************
void CRenderView::OnTimer(UINT nIDEvent) 
{
	switch(nIDEvent)
		{
		case 0:
			break;
		// Rotation
		case 1:
			m_yRotation += 5.0f;
			InvalidateRect(NULL,FALSE);
			break;
		default:
			{}
		}
}


void CRenderView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags) 
{
	static int GraviteON=FALSE;

	if(nChar==' ') // Space bar, gravity management
	{
		if (!GraviteON)
		{
			Rein->gravite[0] = 0.;
			Rein->gravite[1] = -0.098; 
			Rein->gravite[2] = 0.;
			GraviteON=TRUE;
		}
		else
		{
			Rein->gravite[0] = 0.;
			Rein->gravite[1] = 0.;
			Rein->gravite[2] = 0.;
			GraviteON=FALSE;
		}

	InvalidateRect(NULL,FALSE);
	}

	if(nChar=='H')
	{
	Rein->Curs_Pos[0]+=0.1f;

	}
	if(nChar=='G')
	{
	Rein->Curs_Pos[0]-=0.1f;
	}
	if(nChar=='Y')
	{
	Rein->Curs_Pos[1]+=0.1f;
	}
	if(nChar=='B')
	{
	Rein->Curs_Pos[1]-=0.1f;
	}
	if(nChar=='N')
	{
	Rein->Curs_Pos[2]+=0.1f;
	}
	if(nChar=='T')
	{
	Rein->Curs_Pos[2]-=0.1f;
	}
	
	if(nChar=='A')
	{
	//sphere_z-=0.1f;
	}
	if (nChar=='C')
	{
		m_SphereMove=!m_SphereMove;
	}
	if (nChar=='Z')
	{
		booltestCol=!booltestCol;
	}
	if (nChar=='1')
	{
		Rein->firstCapture=true;
	}
	if (nChar=='2')
	{
		//Rein->Capture=true;
		Rein->firstCapture=false;
	}

	// Indentation - compression/extension
	if ((nChar=='3')&&(!Rein->Indent)&&(!Rein->StopIndent))
	{
		//if(Rein->FilePath==(CString)"cube_unique\\cubesym") // Indentation pour cube_unique seulement
		Rein->StartIndent=true; 
	}
	if (nChar=='4')
	{
		Rein->StopIndent=true;
	}

	if (nChar=='P')
	{
		if(Rein->PointAffiche==(Rein->numVertices-1))
			Rein->PointAffiche=0;
		else
            Rein->PointAffiche++;
	}
	if (nChar=='M')
	{
		if(Rein->PointAffiche==0)
			Rein->PointAffiche=Rein->numVertices-1;
		else
            Rein->PointAffiche--;
	}

	if(nChar=='O')
	{
		Rein->yIndent_sup=Rein->yIndent_sup+0.1;
	}
	if(nChar=='L')
	{
		Rein->yIndent_sup=Rein->yIndent_sup-0.1;
	}

	if (nChar=='9')
	{
//		pFormCommandView->m_time1 = Rein->sommetCollision;
//				pFormCommandView->UpdateData(FALSE);
	}

	//Rein
	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}



/////////////////////////////////////////////////////////////////////////////
// FormCommandView

IMPLEMENT_DYNCREATE(CFormCommandView, CFormView)

CFormCommandView::CFormCommandView()
: CFormView(CFormCommandView::IDD)
{
	//{{AFX_DATA_INIT(CFormCommandView)
	m_Lighting = FALSE;
	m_Smooth = TRUE;
	m_Antialias = FALSE;
	m_Model = 1;
//	m_VRotate = FALSE;
	m_time1 = 0.0;
	m_time2 = 0.0;
	m_time3 = 0.0;
	m_time4 = 0.0;
	//}}AFX_DATA_INIT
}
CFormCommandView::~CFormCommandView()
{
}
void CFormCommandView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CFormCommandView)
	DDX_Control(pDX, IDC_SLIDER_X, m_SliderScaleX);
//	DDX_Control(pDX, IDC_FRAME_COLOR_LIGHT_AMBIENT, m_ControlColorLightAmbient);
//	DDX_Control(pDX, IDC_FRAME_COLOR_BACK, m_ControlBackColor);
	DDX_Check(pDX, IDC_CHECK_LIGHTING, m_Lighting);
	DDX_Check(pDX, IDC_CHECK_SMOOTH, m_Smooth);
	DDX_Check(pDX, IDC_CHECK_ANTIALIAS, m_Antialias);
	DDX_Radio(pDX, IDC_RADIO_MODEL_0, m_Model);
//	DDX_Check(pDX, IDC_CHECK_VROTATION, m_VRotate);
	DDX_Text(pDX, IDC_TIME1, m_time1);
//	DDX_Text(pDX, IDC_TIME2, m_time2);
//	DDX_Text(pDX, IDC_TIME3, m_time3);
	DDX_Text(pDX, IDC_TIME4, m_time4);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CFormCommandView, CFormView)
	//{{AFX_MSG_MAP(CFormCommandView)
	ON_WM_PAINT()
	ON_WM_LBUTTONUP()
	ON_BN_CLICKED(IDC_RADIO_MODEL_0, OnRadioModel0)
	ON_BN_CLICKED(IDC_RADIO_MODEL_1, OnRadioModel1)
	ON_BN_CLICKED(IDC_RADIO_MODEL_2, OnRadioModel2)
	ON_BN_CLICKED(IDC_CHECK_LIGHTING, OnCheckLighting)
	ON_BN_CLICKED(IDC_CHECK_SMOOTH, OnCheckSmooth)
//	ON_BN_CLICKED(IDC_CHECK_VROTATION, OnCheckVrotation)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_CHECK_LINK_SCALE, OnCheckLinkScale)
	ON_BN_CLICKED(IDC_CHECK_ANTIALIAS, OnCheckAntialias)
	//}}AFX_MSG_MAP
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER_X, OnNMCustomdrawSliderX)
	ON_EN_CHANGE(IDC_TIME3, OnEnChangeTime3)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CFormCommandView diagnostics

#ifdef _DEBUG
void CFormCommandView::AssertValid() const
{
	CFormView::AssertValid();
}

void CFormCommandView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}

CToolDoc* CFormCommandView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CToolDoc)));
	return (CToolDoc*)m_pDocument;
}

#endif //_DEBUG

//********************************************
// OnPaint
//********************************************
void CFormCommandView::OnPaint() 
{
	// Device context for painting
	CPaintDC dc(this); 
	
	// Options are stored in Application
	CToolApp *pApp = (CToolApp *)AfxGetApp();
	CRect rect;
/*
	// Color back
	m_ControlBackColor.GetWindowRect(&rect);	
	ScreenToClient(&rect);
	CBrush BrushBack(pApp->m_OptionColorGlBack);
	dc.FillRect(&rect,&BrushBack);
	
	// Color light ambient
	m_ControlColorLightAmbient.GetWindowRect(&rect);	
	ScreenToClient(&rect);
	CBrush BrushLightAmbient(pApp->m_OptionColorGlLightAmbient);
	dc.FillRect(&rect,&BrushLightAmbient);
*/
}

//********************************************
// OnLButtonUp
//********************************************
void CFormCommandView::OnLButtonUp(UINT nFlags, CPoint point) 
{
	CRect rect;
	CToolApp *pApp = (CToolApp *)AfxGetApp();
/*
	// Option back color
	m_ControlBackColor.GetWindowRect(&rect);	
	ScreenToClient(&rect);
	if(rect.PtInRect(point))
	{
		CColorDialog dlg(pApp->m_OptionColorGlBack);
		if(dlg.DoModal()==IDOK)
		{
			pApp->m_OptionColorGlBack = dlg.GetColor();	
			CRenderView *pView = (CRenderView *)GetRenderView();
			pView->m_ClearColorRed   = (float)GetRValue(pApp->m_OptionColorGlBack) / 255.0f;
			pView->m_ClearColorGreen = (float)GetGValue(pApp->m_OptionColorGlBack) / 255.0f;
			pView->m_ClearColorBlue  = (float)GetBValue(pApp->m_OptionColorGlBack) / 255.0f;
			this->InvalidateRect(&rect,FALSE);
			pView->InvalidateRect(NULL,FALSE); 
		}
	}
	
	// Option ambient light color
	m_ControlColorLightAmbient.GetWindowRect(&rect);	
	ScreenToClient(&rect);
	if(rect.PtInRect(point))
	{
		CColorDialog dlg(pApp->m_OptionColorGlLightAmbient);
		if(dlg.DoModal()==IDOK)
		{
			pApp->m_OptionColorGlLightAmbient = dlg.GetColor();	
			CRenderView *pView = (CRenderView *)GetRenderView();

			// Refresh Light0
			float r = (float)GetRValue(pApp->m_OptionColorGlLightAmbient) / 255.0f;
			float g = (float)GetGValue(pApp->m_OptionColorGlLightAmbient) / 255.0f;
			float b = (float)GetBValue(pApp->m_OptionColorGlLightAmbient) / 255.0f;
			float	ambientProperties[]  = {r,g,b,1.0f};
			glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties);

			// Refresh views
			this->InvalidateRect(&rect,FALSE);
			pView->InvalidateRect(NULL,FALSE); 

		}
	}
*/	
	CFormView::OnLButtonUp(nFlags, point);
}

//********************************************
// GetRenderView
//********************************************
CView *CFormCommandView::GetRenderView() 
{
	CToolApp *pApp = (CToolApp *)AfxGetApp();
	CMainFrame *pMainFrame = (CMainFrame *)pApp->m_pMainWnd;
	CChildFrame *pFrame = (CChildFrame *)pMainFrame->GetActiveFrame();
	
	CView *pView = (CView *)pFrame->m_wndSplitter.GetPane(0,1);
	return pView;
}
//********************************************
// Model
//********************************************
void CFormCommandView::OnRadioModel0() 
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
	this->GetRenderView()->InvalidateRect(NULL,FALSE); 
}

void CFormCommandView::OnRadioModel1() 
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	this->GetRenderView()->InvalidateRect(NULL,FALSE); 
}

void CFormCommandView::OnRadioModel2() 
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	this->GetRenderView()->InvalidateRect(NULL,FALSE); 
}

//********************************************
// OnCheckLighting
//********************************************
void CFormCommandView::OnCheckLighting() 
{
	m_Lighting = !m_Lighting;
	if(m_Lighting)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);
	this->GetRenderView()->InvalidateRect(NULL,FALSE); 
}

//********************************************
// OnCheckSmooth
//********************************************
void CFormCommandView::OnCheckSmooth() 
{
	m_Smooth = !m_Smooth;
	if(m_Smooth)
		glShadeModel(GL_SMOOTH);
	else
		glShadeModel(GL_FLAT);
	this->GetRenderView()->InvalidateRect(NULL,FALSE); 
}
/*
void CFormCommandView::OnCheckVrotation() 
{
	m_VRotate = !m_VRotate;
	CRenderView *pView = (CRenderView *)GetRenderView();
	if(m_VRotate)
		pView->SetTimer(1,10,NULL);
	else
		pView->KillTimer(1);
}
*/

//********************************************
// OnInitialUpdate
//********************************************
void CFormCommandView::OnInitialUpdate() 
{
	CFormView::OnInitialUpdate();
	
	// Dimensionnement des fenêtres
//	CRect Rect;
//	AfxGetMainWnd()->GetClientRect(&Rect);
//	int x1 = Rect.left;
//	int y1 = Rect.top;
	int x1 = 50;
	int y1 = 50;
	int xwidth=900;
	int yheight=750;
	AfxGetMainWnd()->SetWindowPos(NULL,x1,y1,xwidth,yheight,SWP_FRAMECHANGED);

	WINDOWPLACEMENT lpwndpl;
	GetParentFrame()->GetWindowPlacement(&lpwndpl);
	lpwndpl.showCmd=SW_SHOWMAXIMIZED;
	GetParentFrame()->SetWindowPlacement(&lpwndpl);

	// Slider
	TRACE("Sliders : updating...\n");

	m_SliderScaleX.SetRange(0,100,TRUE);
	m_SliderScaleX.SetPos(50);

//	char time[80] = "Hello time !";
//	m_Time = (CString)(time);
//	m_Time = _T("Hello time !");
//	m_Time.SetString("Hello time !");
//	DoDataExchange(m_Time);

//	CRenderView *pView = (CRenderView *)GetRenderView();

	m_time1 = 100;
	UpdateData(FALSE);
}

//********************************************
// OnHScroll
//********************************************
void CFormCommandView::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	switch( nSBCode )
	{
	case TB_BOTTOM:
		UpdateScale();
		break;
	case TB_ENDTRACK:
		UpdateScale();
		break;
	case TB_LINEDOWN:
		UpdateScale();
		break;
	case TB_LINEUP:
		UpdateScale();
		break;
	case TB_PAGEDOWN:
		UpdateScale();
		break;
	case TB_PAGEUP:
		UpdateScale();
		break;
	case TB_THUMBPOSITION:
		UpdateScale();
		break;
	case TB_THUMBTRACK:
		UpdateScale();
		break;
	case TB_TOP:
		UpdateScale();
		break;
	default:
		{}
	}

	// Update scaling
	GetRenderView()->InvalidateRect(NULL,FALSE); 
	
	CFormView::OnHScroll(nSBCode, nPos, pScrollBar);
}

BOOL CFormCommandView::UpdateScale() 
{
	CRenderView *pView = (CRenderView *)GetRenderView();
	
	pView->m_xScaling = (float)m_SliderScaleX.GetPos()*pView->m_zoomslidefactor;

	return TRUE;
}

void CFormCommandView::OnCheckLinkScale() 
{
/*
	m_LinkScale = !m_LinkScale;	

	if(m_LinkScale)
	{
		CRenderView *pView = (CRenderView *)GetRenderView();
		m_SliderScaleY.SetPos(m_SliderScaleX.GetPos());
		m_SliderScaleZ.SetPos(m_SliderScaleX.GetPos());
		pView->m_yScaling = pView->m_zScaling = pView->m_xScaling;
	}

	m_SliderScaleY.EnableWindow(!m_LinkScale);
	m_SliderScaleZ.EnableWindow(!m_LinkScale);
	GetRenderView()->InvalidateRect(NULL,FALSE); 
*/

//	m_LinkScale = TRUE;	

	CRenderView *pView = (CRenderView *)GetRenderView();
//	m_SliderScaleY.SetPos(m_SliderScaleX.GetPos());
//	m_SliderScaleZ.SetPos(m_SliderScaleX.GetPos());
//	pView->m_yScaling = pView->m_zScaling = pView->m_xScaling;

//	m_SliderScaleY.EnableWindow(FALSE);
//	m_SliderScaleZ.EnableWindow(FALSE);
	GetRenderView()->InvalidateRect(NULL,FALSE); 

}

//********************************************
// OnCheckAntialias
// Toggle antialiased lines
//********************************************
void CFormCommandView::OnCheckAntialias() 
{
	m_Antialias = !m_Antialias;

	if(m_Antialias)
	{
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
		glLineWidth(1.5);
	}
	else
	{
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
		glLineWidth(1.0);
	}
	GetRenderView()->InvalidateRect(NULL,FALSE); 
}

void CFormCommandView::OnNMCustomdrawSliderX(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO : ajoutez ici le code de votre gestionnaire de notification de contrôle
	*pResult = 0;
}

void CFormCommandView::OnEnChangeTime3()
{
	// TODO :  S'il s'agit d'un contrôle RICHEDIT, le contrôle
	// n'enverra la notification que si vous substituez la fonction CFormView::OnInitDialog()
	// et l'appel CRichEditCtrl().SetEventMask()
	// par l'indicateur ENM_CHANGE assorti de l'opérateur OR dans le masque.

	// TODO :  Ajoutez ici le code de votre gestionnaire de notification de contrôle
}


///////MainFrm
/////////////////////////////////////////////////////////////////////////////
// CMainFrame

IMPLEMENT_DYNAMIC(CMainFrame, CMDIFrameWnd)
BEGIN_MESSAGE_MAP(CMainFrame, CMDIFrameWnd)
	//{{AFX_MSG_MAP(CMainFrame)
	ON_WM_CREATE()
	ON_WM_PAINT()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // status line indicator
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};

/////////////////////////////////////////////////////////////////////////////
// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{
	// TODO: add member initialization code here
}
CMainFrame::~CMainFrame()
{
}
int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CMDIFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
/*
	if (!m_wndToolBar.Create(this) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}
	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
		  sizeof(indicators)/sizeof(UINT)))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}

	// TODO: Remove this if you don't want tool tips or a resizeable toolbar
	m_wndToolBar.SetBarStyle(m_wndToolBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC);

	// TODO: Delete these three lines if you don't want the toolbar to
	//  be dockable
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndToolBar);
*/
	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	return CMDIFrameWnd::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CMDIFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CMDIFrameWnd::Dump(dc);
}

#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CMainFrame message handlers

void CMainFrame::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
}


/////////DlgAbout
/////////////////////////////////////////////////////////////////////////////
// CToolApp

BEGIN_MESSAGE_MAP(CToolApp, CWinApp)
	//{{AFX_MSG_MAP(CToolApp)
	ON_COMMAND(ID_APP_ABOUT, OnAppAbout)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
	// Standard file based document commands
	ON_COMMAND(ID_FILE_NEW, CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, CWinApp::OnFileOpen)
	// Standard print setup command
	ON_COMMAND(ID_FILE_PRINT_SETUP, CWinApp::OnFilePrintSetup)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CToolApp construction

CToolApp::CToolApp()
{
	// Options
	m_OptionColorGlBack = RGB(0,0,0);   //m_OptionColorGlBack = RGB(255,255,255);
	// ****** TODO ******
	m_OptionColorGlLightAmbient = RGB(200,200,200); 
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CToolApp object

CToolApp theApp;

/////////////////////////////////////////////////////////////////////////////
// CToolApp initialization

BOOL CToolApp::InitInstance()
{
	AfxEnableControlContainer();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	//  of your final executable, you should remove from the following
	//  the specific initialization routines you do not need.

#ifdef _AFXDLL
	Enable3dControls();			// Call this when using MFC in a shared DLL
#else
	Enable3dControlsStatic();	// Call this when linking to MFC statically
#endif

	// Change the registry key under which our settings are stored.
	// You should modify this string to be something appropriate
	// such as the name of your company or organization.
	SetRegistryKey(_T("3D Toolbox"));

	LoadStdProfileSettings(10);  // Load standard INI file options (including MRU)

	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views.

	CMultiDocTemplate* pDocTemplate;
	pDocTemplate = new CMultiDocTemplate(
		IDR_MODELTYPE,
		RUNTIME_CLASS(CToolDoc),
		RUNTIME_CLASS(CChildFrame), // custom MDI child frame
		RUNTIME_CLASS(CRenderView));
	AddDocTemplate(pDocTemplate);

	// create main MDI Frame window
	CMainFrame* pMainFrame = new CMainFrame;
	if (!pMainFrame->LoadFrame(IDR_MAINFRAME))
		return FALSE;
	m_pMainWnd = pMainFrame;
	//pMainFrame->SetTitle("Texte 1");

	// Enable drag/drop open
	m_pMainWnd->DragAcceptFiles();

	// Enable DDE Execute open
	EnableShellOpen();
	RegisterShellFileTypes(TRUE);

	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);

	// Dispatch commands specified on the command line
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// Load options
	LoadOptions();

	// The main window has been initialized, so show and update it.
	pMainFrame->ShowWindow(m_nCmdShow);
//	pMainFrame->ShowWindow(SW_SHOW);
	pMainFrame->UpdateWindow();

	return TRUE;
}

// App command to run the dialog
void CToolApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

/////////////////////////////////////////////////////////////////////////////
// CToolApp commands

int CToolApp::ExitInstance() 
{
	SaveOptions();	
	return CWinApp::ExitInstance();
}

//**************************************************
// LoadOptions
// Load from registry
//**************************************************
BOOL CToolApp::LoadOptions()
{
	unsigned int red,green,blue;

	// Gl back color
	red = GetProfileInt("OpenGL back color","Red",0);
	green = GetProfileInt("OpenGL back color","Green",0);
	blue = GetProfileInt("OpenGL back color","Blue",0);
	m_OptionColorGlBack = RGB(red,green,blue);

	// Gl light ambient color
	red = GetProfileInt("OpenGL light ambient color","Red",200);
	green = GetProfileInt("OpenGL light ambient color","Green",200);
	blue = GetProfileInt("OpenGL light ambient color","Blue",200);
	m_OptionColorGlLightAmbient = RGB(red,green,blue);
	
	return TRUE;
}

//**************************************************
// SaveOptions
// Load from registry
//**************************************************
BOOL CToolApp::SaveOptions()
{
	unsigned char red,green,blue;

	// Gl back color
	red = GetRValue(m_OptionColorGlBack);
	green = GetGValue(m_OptionColorGlBack);
	blue = GetBValue(m_OptionColorGlBack);
	WriteProfileInt("OpenGL back color","Red",red);
	WriteProfileInt("OpenGL back color","Green",green);
	WriteProfileInt("OpenGL back color","Blue",blue);
	
	// Gl light ambient color
	red = GetRValue(m_OptionColorGlLightAmbient);
	green = GetGValue(m_OptionColorGlLightAmbient);
	blue = GetBValue(m_OptionColorGlLightAmbient);
	WriteProfileInt("OpenGL light ambient color","Red",red);
	WriteProfileInt("OpenGL light ambient color","Green",green);
	WriteProfileInt("OpenGL light ambient color","Blue",blue);
	
	return TRUE;
}


/////////////////////////////////////////////////////////////////////////////
// CToolDoc

IMPLEMENT_DYNCREATE(CToolDoc, CDocument)

BEGIN_MESSAGE_MAP(CToolDoc, CDocument)
	//{{AFX_MSG_MAP(CToolDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CToolDoc construction/destruction

//********************************************
// Constructor
//********************************************
CToolDoc::CToolDoc()
{
}

//********************************************
// Destructor
//********************************************
CToolDoc::~CToolDoc()
{
}

BOOL CToolDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
// CToolDoc serialization

void CToolDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}

/////////////////////////////////////////////////////////////////////////////
// CToolDoc diagnostics

#ifdef _DEBUG
void CToolDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CToolDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CToolDoc commands


//////////////////////////////////////////////
//////////////////////////////////////////////
// RENDERING
//////////////////////////////////////////////
//////////////////////////////////////////////

//***********************************************
// RenderScene
//***********************************************
void CToolDoc::RenderScene()
{
	// Default rendering 
/*
	glColor3ub(255,255,255);
	GLUquadricObj*	q = gluNewQuadric();
	gluQuadricDrawStyle(q, GLU_FILL);
	gluQuadricNormals(q, GLU_SMOOTH);
	gluSphere(q, 1.0, 16, 16);
	gluDeleteQuadric(q);

	float width = 1.0f;
	float step = 0.1f;

	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	for(x = -width; x < width; x += step)
		{
			glBegin(GL_QUAD_STRIP); 
			for(z = -width; z < width; z += step)
			{
				float r,g,b;
				g = 0.0f;

				y = (float)sin(3*x)*cos(3*z)/3.0f;
				r = y*3.0f;
				b = 1.0f-y;
				glColor3f(r,g,b);
				glVertex3d(x,y,z);

				y = (float)sin(3*(x+step))*cos(3*z)/3.0f;
				r = y*3.0f;
				b = 1.0f-y;
				glColor3f(r,g,b);
				glVertex3d(x+step,y,z);
			}
			glEnd();
		}
*/
}


//DlgAbout
CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// ChildFrame
IMPLEMENT_DYNCREATE(CChildFrame, CMDIChildWnd)

BEGIN_MESSAGE_MAP(CChildFrame, CMDIChildWnd)
	//{{AFX_MSG_MAP(CChildFrame)
	ON_WM_PAINT()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()
/////////////////////////////////////////////////////////////////////////////
// CChildFrame construction/destruction

CChildFrame::CChildFrame()
{
	// TODO: add member initialization code here
}
CChildFrame::~CChildFrame()
{
}
//********************************************
// OnCreateClient
//********************************************
BOOL CChildFrame::OnCreateClient( LPCREATESTRUCT /*lpcs*/,
	CCreateContext* pContext)
{
	if (!m_wndSplitter.CreateStatic(this, 1, 2,WS_CHILD | WS_VISIBLE))
//	if (!m_wndSplitter.CreateStatic(this, 1, 2))
	{
		TRACE("Failed to CreateStaticSplitter\n");
		return FALSE;
	}
  // First splitter pane
  if (!m_wndSplitter.CreateView(0, 0,
      RUNTIME_CLASS(CFormCommandView), CSize(163,50), pContext)) //200,100
  {
		TRACE("Failed to create command view pane\n");
    return FALSE;
  }
  // Second splitter pane
  if (!m_wndSplitter.CreateView(0, 1,
      RUNTIME_CLASS(CRenderView), CSize(163,50), pContext)) //200,400
  {
    TRACE("Failed to create preview pane\n");
    return FALSE;
  }
	return TRUE;
}

BOOL CChildFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
//SetTitle("bliiiiiiblo");
	return CMDIChildWnd::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// CChildFrame diagnostics
#ifdef _DEBUG
void CChildFrame::AssertValid() const
{
	CMDIChildWnd::AssertValid();
}
void CChildFrame::Dump(CDumpContext& dc) const
{
	CMDIChildWnd::Dump(dc);
}
#endif //_DEBUG
/////////////////////////////////////////////////////////////////////////////
// CChildFrame message handlers
void CChildFrame::OnPaint() 
{
	CPaintDC dc(this); 
}
