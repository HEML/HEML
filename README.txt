HEML_OpenSource version 1.0 - March 2016
========================================
* licence terms: GNU GENERAL PUBLIC LICENSE, Version 3, 29 June 2007 - see License text

* This source code implements the HEML method described in 2015:
F. Goulette, Z.-W. Chen. Fast computation of soft tissue deformations in real-time simulation with Hyper-Elastic Mass Links. Computer Methods in Applied Mechanics and Engineering, 295, 18-38, 2015.
doi:10.1016/j.cma.2015.06.015
http://www.sciencedirect.com/science/article/pii/S0045782515002078

Please cite this reference when publishing research work using this HEML Open Source code, original or derivated.

* A preliminary version of the method was published in 2006 in:
F. Goulette, S. Chendeb. A Framework for Fast Computation of Hyper-Elastic Materials Deformations in Real-Time Simulation of Surgery. Computational Biomechanics for Medicine (CBM) Workshop of the Medical Image Computing and Computer Assisted Intervention (MICCAI) Conference, October 2006, Copenhagen, Denmark.
https://hal.archives-ouvertes.fr/hal-01259670

* This work was performed at MINES ParisTech, CAOR-Robotics Lab, 2006-2016



Directories and files description
=================================

-Four folders are included:
	*dll: include .dll files
	*Examples: two cubes and one kidney numerical examples are included,
	*res: some components need for visualisation
	*src: include source code 

-Three text files:
	*config.txt
	*README.txt
	*LICENSE.txt

-Two MS Visual Studio project files:
	*Tool.vcxproj
	*Tool.sln

-One executable file Tool.exe (compiled for Windows 7)


Documentation of the two main codes 'HEML.cpp' and 'View_HEML.cpp' is included directly in these files.


To launch the programme (stand-alone - for Windows 7)
=====================================================

Launch the "Tool.exe" executable file at the root of the directory.

The HEML code is launched according to the configuration indicated in the config.txt file placed in the same directory. The config.txt provides 3 examples.



To build the solution
======================

Use MS Visual Studio 2012 (v110) in Platform Toolset;
Use MFC in a Shared DLL;
Additional Dependencies: opengl32.lib, glu32.lib, kernel32.lib.

In "Build" menu, click "Build Tool". 
This generates the Debug directory with intermediate compiled files, 
and the Tool.exe executable file at the root of the directory.

To launch the programme:
Once the solution is built (see above),
in "Debug" menu, click "Start Without Debugging"

 

For interaction
================

* Examples "Complex Cube" and "Simple Cube"
1 : grip the edge of the cube
2 : release

C : move cursor / object (mouse mouvement)

* Example "Kidney"
SPACE BAR:  free fall of the object towards a barrier plane



Input file
==========

The file '.t' is the input file, it includes the number of nodes and elements, the coordinates of all the nodes, and the node number of each element.
It should be notices that the number of the nodes begins with 0, not 1.



Configuration and parameters 
============================

* Management of Saint Venant-Kirchhoff, Neo-Hookean and Monney-Rivlin material 
parameters, damping parameters, time step, density and mesh file (file config.txt)
// MATERIAL_TYPE : 1=MS3D (Mass-Spring 3D), 
	 	   2=STVK (Saint Venant-Kirchhoff), 
		   3=NH (Néo-Hookéen),
	           4=Mooney (Mooney-Rivlin)
  their parameters:
	Saint Venant-Kirchhoff : E,Nu (Lambda = (Nu*E)/((1.-2.*Nu)*(1.+Nu)); Mu = E/(2.*(1.+Nu));
	Néo-Hookéen : mu0(mu0=2*c1),lambda0
	Mooney-Rivlin : c01, c10
   

* You can define which node to be attached to the interactive position of the sphere in "HEML.cpp", function "CalculForces_et_Integration()": sommet[][]=xSphere/ySphere/zSphere.


Examples
========

There are three examples: SimpleCube, CompliCube and Kidney. 
For each example, the different parameters are shown in the "config.txt" file.


