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

#if !defined(AFX_STDAFX_H__06A3562B_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_STDAFX_H__06A3562B_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#include <afxdisp.h>        // MFC OLE automation classes
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT


//////////////////////////////////////////////
//////////////////////////////////////////////
// Do not care OpenGL Headers...
//////////////////////////////////////////////
//////////////////////////////////////////////
#include <gl\gl.h>
#include <gl\glu.h>

#endif // !defined(AFX_STDAFX_H__06A3562B_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)




////////////DlgAbout
#ifndef _DIALOG_ABOUT_
#define _DIALOG_ABOUT_

//Tool
#if !defined(AFX_TOOL_H__06A35629_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_TOOL_H__06A35629_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

//Resource
#define IDD_ABOUTBOX                    100
#define IDR_MAINFRAME                   128
#define IDR_MODELTYPE                   129
#define IDC_CURSOR_ROTATION             131
#define IDD_FORM_COMMAND                150
#define IDC_FRAME_COLOR_BACK            1000
#define IDC_CHECK_ANTIALIAS             1001
#define IDC_CHECK_SMOOTH                1002
#define IDC_RADIO_MODEL_1               1003
#define IDC_CHECK_LIGHTING              1004
#define IDC_RADIO_MODEL_2               1005
#define IDC_RADIO_MODEL_0               1006
#define IDC_CHECK_VROTATION             1007
#define IDC_SLIDER_X                    1008
#define IDC_CHECK_LINK_SCALE            1009
#define IDC_FRAME_COLOR_LIGHT_AMBIENT   1010
#define IDC_SLIDER_Y                    1011
#define IDC_SLIDER_Z                    1012
#define IDC_FRAME_COLOR_LIGHT_AMBIENT2  1013
#define IDC_FRAME_COLOR_LIGHT_AMBIENT3  1014
#define IDC_TIME                        1015
#define IDC_TIME1                       1017
#define IDC_TIME2                       1019
#define IDC_TIME3                       1020
#define IDC_TIME4                       1021

// Next default values for new objects
// 
#ifdef APSTUDIO_INVOKED
#ifndef APSTUDIO_READONLY_SYMBOLS
#define _APS_3D_CONTROLS                     1
#define _APS_NEXT_RESOURCE_VALUE        132
#define _APS_NEXT_COMMAND_VALUE         32771
#define _APS_NEXT_CONTROL_VALUE         1022
#define _APS_NEXT_SYMED_VALUE           101
#endif
#endif

/////////////////////////////////////////////////////////////////////////////
// CToolApp:
// See Tool.cpp for the implementation of this class
//

class CToolApp : public CWinApp
{
// Options
public:

	// Registry keys
	BOOL LoadOptions();
	BOOL SaveOptions();

	COLORREF m_OptionColorGlBack;
	COLORREF m_OptionColorGlLightAmbient;
public:
	CToolApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CToolApp)
	public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CToolApp)
	afx_msg void OnAppAbout();
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};
/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TOOL_H__06A35629_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)


//ToolDoc
#if !defined(AFX_TOOLDOC_H__06A35631_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_TOOLDOC_H__06A35631_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

class CSeneGraph3d; 

class CToolDoc : public CDocument
{
protected: // create from serialization only
	CToolDoc();
	DECLARE_DYNCREATE(CToolDoc)

// Attributes
private:
	//CSeneGraph3d m_SceneGraph;

// Functions
public:

	// OpenGL
	void RenderScene();

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CToolDoc)
	public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CToolDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CToolDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TOOLDOC_H__06A35631_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)

//DlgAbout
class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
		// No message handlers
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};
#endif // _DIALOG_ABOUT_

//ChildFrm
#if !defined(AFX_CHILDFRM_H__06A3562F_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_CHILDFRM_H__06A3562F_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

class CChildFrame : public CMDIChildWnd
{
	DECLARE_DYNCREATE(CChildFrame)
public:
	CChildFrame();
// Attributes
public:
	CSplitterWnd m_wndSplitter;
public:
// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CChildFrame)
	public:
	virtual BOOL OnCreateClient(LPCREATESTRUCT lpcs, CCreateContext* pContext);
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CChildFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// Generated message map functions
protected:
	//{{AFX_MSG(CChildFrame)
	afx_msg void OnPaint();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CHILDFRM_H__06A3562F_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)





///////////////////RenderView
#if !defined(AFX_RENDERVIEW_H__06A35633_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_RENDERVIEW_H__06A35633_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
class CToolDoc;

class CFormCommandView;

class CRenderView : public CView
{
protected: // create from serialization only
	CRenderView();
	DECLARE_DYNCREATE(CRenderView)

// Attributes
public:
	CToolDoc* GetDocument();
	CFormCommandView* GetFormCommandView(); 

// Operations
public:
	CFormCommandView* pFormCommandView;

	// OpenGL specific
	BOOL SetWindowPixelFormat(HDC hDC);
	BOOL CreateViewGLContext(HDC hDC);
	HGLRC m_hGLContext;
	int m_GLPixelIndex;

	// Mouse 
	BOOL m_LeftButtonDown;
	BOOL m_RightButtonDown;
	CPoint m_LeftDownPos;
	CPoint m_RightDownPos;
	HCURSOR m_CursorRotation;
	BOOL m_SphereMove;

	// Position, rotation ,scaling
	void InitGeometry(void);

	float m_xRotation;
	float m_yRotation;
	float m_zRotation;

	float m_xTranslation;
	float m_yTranslation;
	float m_zTranslation;
	float m_xScaling;
	float m_zoomslidefactor;
	int	  m_slideinit;
//	float m_yScaling;
//	float m_zScaling;

	// Colors
	float m_ClearColorRed;
	float m_ClearColorGreen;

	float m_ClearColorBlue;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRenderView)
	public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CRenderView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CRenderView)
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnPaint();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in RenderView.cpp
inline CToolDoc* CRenderView::GetDocument()
   { return (CToolDoc*)m_pDocument; }
#endif

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_RENDERVIEW_H__06A35633_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)



//FormCommandView
#if !defined(AFX_FORMCOMMANDVIEW_H__A4283CC1_72E6_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_FORMCOMMANDVIEW_H__A4283CC1_72E6_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

/////////////////////////////////////////////////////////////////////////////
// CFormCommandView form view

#ifndef __AFXEXT_H__
#include <afxext.h>
#endif

class CToolDoc;

class CFormCommandView : public CFormView
{
protected:
	CFormCommandView();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(CFormCommandView)

// Form Data
public:
	//{{AFX_DATA(CFormCommandView)
	enum { IDD = IDD_FORM_COMMAND };
	CSliderCtrl	m_SliderScaleX;
//	CStatic	m_ControlColorLightAmbient;
//	CStatic	m_ControlBackColor;
	BOOL	m_Lighting;
	BOOL	m_Smooth;
	BOOL	m_Antialias;
	int		m_Model;
//	BOOL	m_VRotate;
	float	m_time1;
	double	m_time2;
	double	m_time3;
	double	m_time4;
	//}}AFX_DATA

// Attributes
public:
	CToolDoc* GetDocument();
	CView *GetRenderView(); 
	BOOL UpdateScale(); 

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CFormCommandView)
	public:
	virtual void OnInitialUpdate();
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	virtual ~CFormCommandView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
	//{{AFX_MSG(CFormCommandView)
	afx_msg void OnPaint();
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRadioModel0();
	afx_msg void OnRadioModel1();
	afx_msg void OnRadioModel2();
	afx_msg void OnCheckLighting();
	afx_msg void OnCheckSmooth();
//	afx_msg void OnCheckVrotation();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnCheckLinkScale();
	afx_msg void OnCheckAntialias();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnNMCustomdrawSliderX(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnEnChangeTime3();
};

#ifndef _DEBUG  // debug version in RenderView.cpp
inline CToolDoc* CFormCommandView::GetDocument()
   { return (CToolDoc*)m_pDocument; }
#endif

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_FORMCOMMANDVIEW_H__A4283CC1_72E6_11D1_A6C2_00A0242C0A32__INCLUDED_)


//////MainFrm
#if !defined(AFX_MAINFRM_H__06A3562D_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)
#define AFX_MAINFRM_H__06A3562D_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

class CMainFrame : public CMDIFrameWnd
{
	DECLARE_DYNAMIC(CMainFrame)
public:
	CMainFrame();
// Attributes
public:
// Operations
public:
// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMainFrame)
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:  // control bar embedded members
	CStatusBar  m_wndStatusBar;
	CToolBar    m_wndToolBar;

// Generated message map functions
protected:
	//{{AFX_MSG(CMainFrame)
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_MAINFRM_H__06A3562D_72E5_11D1_A6C2_00A0242C0A32__INCLUDED_)



