#include <GL/glut.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv.hpp"
#include "vector3d.h"
#include "esfera.h"
#include "cubo.h"
#include "cilindro.h"
#include "rosco.h"
#include "compuesto.h"
#include "modelo.h"
#include "escena.h"
#include "pared.h"
#include "camara.h"
#include "textura.h"
#include "rectangulo.h"
#include "fondo_textura.h"
#include "vista.h"
#include "luz.h"
#include "proyeccion_perspectiva.h"
#include "pose_estimation_chessboard.h"
#include <math.h> 

using namespace cv;

double t=0.0;
double dt=1.0/30;

int mx=-1,my=-1;        // Previous mouse coordinates

Mat tablero;

double vel;

//SE cargan todas las variables y objetos necesarios incluyendo las 15 esferas y 11 cubos de todos los niveles
Escena e;
Cubo *pc;
Modelo *m;
Esfera *mainChar;
Textura ladrillos, paredTex, texTablero, testTexture;
Rectangulo *pared;
Esfera* auxEsf1;
Esfera* auxEsf2;
Esfera* auxEsf3;
Esfera* auxEsf4;
Esfera* auxEsf5;
Esfera* auxEsf6;
Esfera* auxEsf7;
Esfera* auxEsf8;
Esfera* auxEsf9;
Esfera* auxEsf10;
Esfera* auxEsf11;
Esfera* auxEsf12;
Esfera* auxEsf13;
Esfera* auxEsf14;
Esfera* auxEsf15;
Cubo* auxCubo1;
Cubo* auxCubo2;
Cubo* auxCubo3;
Cubo* auxCubo4;
Cubo* auxCubo5;
Cubo* auxCubo6;
Cubo* auxCubo7;
Cubo* auxCubo8;
Cubo* auxCubo9;
Cubo* auxCubo10;
Cubo* auxCubo11;
Cubo* auxCubo12;
Cubo* auxCubo13;
Cubo* auxCuboExtra1;
Cubo* auxCuboExtra2;

Cubo* cuboNeutral1;
Cubo* cuboNeutral2;
Cubo* cuboNeutral3;
Cubo* cuboNeutral4;

//array con cada simbolo asci empleado para dar fluidez al movimiento
bool asciKeyArr[256];

FondoTextura fondo,fondoTablero;

//configuracion de las dos vistas empleadas una con camara TPS y otra con CamaraFly estatica
ProyeccionPerspectiva proyeccion;
vector<Vista> vistas = { { 0.0,0.0,0.5,1,&proyeccion },{ 0.5,0.0,0.5,1,&proyeccion } };
vector<CamaraTPS> camaras(vistas.size());

CamaraFly *mapp;

float getRand(float max,float min=0){
	float n=max-min;
	int ir=rand()%1000;
	return min+n*(float)ir/1000;
}

//calcula determinante de matriz 3x3 pasandole los 9 digitos de la matriz como params
double matrixDet(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33) {
	return (m11 * m22*m33 + m12 * m23*m31 + m13 * m21*m32 - m13 * m22*m31 - m11 * m23*m32 - m12 * m21*m33);
}

//calcula inversa de matriz 3x3 pasandole un array de 9 posiciones (divide la traspuesta de adyacentes entre el determinante)
void matrixInvr(double (&matTest)[9]){//) {
	//double ret[9];
	double IaInverse[9];
	IaInverse[0] = matTest[4] * matTest[8] - matTest[5] * matTest[7];
	IaInverse[1] = -(matTest[3] * matTest[8] - matTest[5] * matTest[6]);
	IaInverse[2] = matTest[3] * matTest[7] - matTest[4] * matTest[6];

	IaInverse[3] = -(matTest[1] * matTest[8] - matTest[2] * matTest[7]);
	IaInverse[4] = matTest[0] * matTest[8] - matTest[2] * matTest[6];
	IaInverse[5] = -(matTest[0] * matTest[7] - matTest[1] * matTest[6]);

	IaInverse[6] = matTest[1] * matTest[5] - matTest[2] * matTest[4];
	IaInverse[7] = -(matTest[0] * matTest[5] - matTest[2] * matTest[3]);
	IaInverse[8] = matTest[0] * matTest[4] - matTest[1] * matTest[3];



	double det = matTest[0] * IaInverse[0] + matTest[1] * IaInverse[1] + matTest[2] * IaInverse[2];

	double aux12 = IaInverse[1];
	double aux13 = IaInverse[2];
	double aux23 = IaInverse[5];

	IaInverse[1] = IaInverse[3];
	IaInverse[2] = IaInverse[6];
	IaInverse[5] = IaInverse[7];

	IaInverse[3] = aux12;
	IaInverse[6] = aux13;
	IaInverse[7] = aux23;

	for (int i = 0; i < 9; i++) {
		matTest[i] = IaInverse[i] / det;
	}
}

//display me
void displayMe(void){
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	//render de primera vista junto a primera camara
	vistas[0].render();
	fondo.render();
    glLoadIdentity();
    camaras[0].render();
    e.render();

	//render de segunda vista junto a segunda camara
	vistas[1].render();
	fondoTablero.render();
    glLoadIdentity();
	mapp->render(); //render de camaraFly mapp
    e.render();
 glutSwapBuffers();
}

//idle
void idle(){
	//aumento de contador de tiempo global
	t+=dt;

	//update de escena y chequeo de collisiones
	e.update(dt);
	e.checkCollision(); 

	//rotacion de esfera
	mainChar->setAngVel(Quaterniond(0.05, mainChar->getVel().crossProduct(Vector3D(0,1,0))));

//varialbes necesarias para calculo de velocidad de esfera principal
 Vector3D mainVel = mainChar->getVel();
 double mainVelLeng = mainVel.length();
 double mainVelInc = 0.1;

 //que desaparezca el "menu de inicio" al cambiar del nivel 0
 if (e.getNivel() != 0) {
	 pared->setRendT(false);
 }

 //update de camara TPS
 CamaraTPS &cam = camaras[0];
 cam.update(dt*vel);
 displayMe();

 //movimientos fluidos conseguidos con el array asci previamente iniciado. Al pulsar una tecla, la posicion correspondiente del array se pone true
 //activando su condicion (en este caso incrementar la velocidad de la esfera)
 if (e.getNivel() != 0) {
	 if (asciKeyArr['w']) {
		 if (mainVel.getZ() < 4)
			 mainChar->setVel(mainVel + Vector3D(0, 0, mainVelInc));
	 }
	 if (asciKeyArr['s']) {
		 if (mainVel.getZ() > -4)
			 mainChar->setVel(mainVel - Vector3D(0, 0, mainVelInc));
	 }
	 if (asciKeyArr['a']) {
		 if (mainVel.getX() > -4)
			 mainChar->setVel(mainVel - Vector3D(mainVelInc, 0, 0));
	 }
	 if (asciKeyArr['d']) {
		 if (mainVel.getX() < 4)
			 mainChar->setVel(mainVel + Vector3D(mainVelInc, 0, 0));
	 }
 }

 //chequeador principal del niveles. Cada vez que se realiza el cambio de nivel (getCambio() == true) aumenta de nivel y inicializa las posiciones, velocidades... correspondientes
 //los objetos del nivel chequeado son posicionados en  la escena y el resto de objetos se quitan. Por ultimo posiciona el personaje principal y establece que ya no se puede
 //cambiar de nivel getCambio() == false
 if (e.getCambio()) {
	 e.setNivel(e.getNivel() + 1);
	 if (e.getNivel() == 0) {


		 auxEsf1->setPos(Vector3D(120, 100, 100));
		 auxEsf2->setPos(Vector3D(110, 100, 100));
		 auxEsf3->setPos(Vector3D(100, 100, 100));

		 auxEsf4->setPos(Vector3D(130, 100, 100));
		 auxEsf5->setPos(Vector3D(140, 100, 100));
		 auxEsf6->setPos(Vector3D(150, 100, 100));
		 auxEsf7->setPos(Vector3D(160, 100, 100));
		 auxCubo1->setPos(Vector3D(-100, 100, -100));

		 auxEsf8->setPos(Vector3D(170, 100, 100));
		 auxEsf9->setPos(Vector3D(180, 100, 100));
		 auxEsf10->setPos(Vector3D(190, 100, 100));
		 auxCubo2->setPos(Vector3D(-110, 100, -100));
		 auxCubo3->setPos(Vector3D(-120, 100, -100));
		 auxCubo4->setPos(Vector3D(-130, 100, -100));

		 auxEsf11->setPos(Vector3D(200, 100, 100));
		 auxCubo5->setPos(Vector3D(-100, 100, -100));
		 auxCubo6->setPos(Vector3D(-100, 100, -100));
		 auxCubo7->setPos(Vector3D(-100, 100, -100));
		 auxCubo8->setPos(Vector3D(-100, 100, -100));

		 auxEsf12->setPos(Vector3D(210, 100, 100));
		 auxEsf13->setPos(Vector3D(220, 100, 100));
		 auxEsf14->setPos(Vector3D(230, 100, 100));
		 auxEsf15->setPos(Vector3D(240, 100, 100));
		 auxCubo9->setPos(Vector3D(-100, 100, -100));
		 auxCubo10->setPos(Vector3D(-100, 100, -100));
		 auxCubo11->setPos(Vector3D(-100, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-200, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-220, 100, -100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));
	 }
	 if(e.getNivel() == 1){
		 auxEsf1->setColMask(1, 1, 1, 2);
		 auxEsf1->setCol(Vector3D(1, 1, 0));
		 auxEsf2->setColMask(1, 1, 1, 2);
		 auxEsf2->setCol(Vector3D(1, 1, 0));
		 auxEsf3->setColMask(1, 1, 1, 2);
		 auxEsf3->setCol(Vector3D(1, 1, 0));

		 auxEsf1->setPos(Vector3D(-5, 5, -0.3));
		 auxEsf1->setVel(Vector3D(1.5, 0, 0));
		 auxEsf1->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf2->setPos(Vector3D(5, 5, 0.3));
		 auxEsf2->setVel(Vector3D(-1.5, 0, 0));
		 auxEsf2->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf3->setPos(Vector3D(-7, 7, -5));
		 auxEsf3->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf4->setPos(Vector3D(130, 100, 100));
		 auxEsf5->setPos(Vector3D(140, 100, 100));
		 auxEsf6->setPos(Vector3D(150, 100, 100));
		 auxEsf7->setPos(Vector3D(160, 100, 100));
		 auxCubo1->setPos(Vector3D(-100, 100, -100));

		 auxEsf8->setPos(Vector3D(170, 100, 100));
		 auxEsf9->setPos(Vector3D(180, 100, 100));
		 auxEsf10->setPos(Vector3D(190, 100, 100));
		 auxCubo2->setPos(Vector3D(-110, 100, -100));
		 auxCubo3->setPos(Vector3D(-120, 100, -100));
		 auxCubo4->setPos(Vector3D(-130, 100, -100));

		 auxEsf11->setPos(Vector3D(200, 100, 100));
		 auxCubo5->setPos(Vector3D(-100, 100, -100));
		 auxCubo6->setPos(Vector3D(-100, 100, -100));
		 auxCubo7->setPos(Vector3D(-100, 100, -100));
		 auxCubo8->setPos(Vector3D(-100, 100, -100));

		 auxEsf12->setPos(Vector3D(210, 100, 100));
		 auxEsf13->setPos(Vector3D(220, 100, 100));
		 auxEsf14->setPos(Vector3D(230, 100, 100));
		 auxEsf15->setPos(Vector3D(240, 100, 100));
		 auxCubo9->setPos(Vector3D(-100, 100, -100));
		 auxCubo10->setPos(Vector3D(-100, 100, -100));
		 auxCubo11->setPos(Vector3D(-100, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-200, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-220, 100, -100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));
	 }
	 if (e.getNivel() == 2) {
		 auxEsf4->setColMask(1, 1, 1, 2);
		 auxEsf4->setCol(Vector3D(1, 1, 0));
		 auxEsf5->setColMask(1, 1, 1, 2);
		 auxEsf5->setCol(Vector3D(1, 1, 0));
		 auxEsf6->setColMask(1, 1, 1, 2);
		 auxEsf6->setCol(Vector3D(1, 1, 0));
		 auxEsf7->setColMask(1, 1, 1, 2);
		 auxEsf7->setCol(Vector3D(1, 1, 0));

		 auxEsf4->setPos(Vector3D(-10, 6, -0.2));
		 auxEsf4->setVel(Vector3D(1.5, 0, 0));
		 auxEsf4->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf5->setPos(Vector3D(10, 4, -0.3));
		 auxEsf5->setVel(Vector3D(-1.5, 0, 0));
		 auxEsf5->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf6->setPos(Vector3D(15, 1, 10));
		 auxEsf6->setFija(true);

		 auxEsf7->setPos(Vector3D(-10, 1, -10));
		 auxEsf7->setFija(true);

		 auxCubo1->setPos(Vector3D(0, 5, 0));
		 //auxCubo1->setAngVel(Quaternion(0.15, Vector3D(0, 0, 1)));
		 auxCubo1->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf1->setPos(Vector3D(120, 100, 100));
		 auxEsf2->setPos(Vector3D(110, 100, 100));
		 auxEsf3->setPos(Vector3D(100, 100, 100));

		 auxEsf8->setPos(Vector3D(170, 100, 100));
		 auxEsf9->setPos(Vector3D(180, 100, 100));
		 auxEsf10->setPos(Vector3D(190, 100, 100));
		 auxCubo2->setPos(Vector3D(-110, 100, -100));
		 auxCubo3->setPos(Vector3D(-120, 100, -100));
		 auxCubo4->setPos(Vector3D(-130, 100, -100));

		 auxEsf11->setPos(Vector3D(200, 100, 100));
		 auxCubo5->setPos(Vector3D(-100, 100, -100));
		 auxCubo6->setPos(Vector3D(-100, 100, -100));
		 auxCubo7->setPos(Vector3D(-100, 100, -100));
		 auxCubo8->setPos(Vector3D(-100, 100, -100));

		 auxEsf12->setPos(Vector3D(210, 100, 100));
		 auxEsf13->setPos(Vector3D(220, 100, 100));
		 auxEsf14->setPos(Vector3D(230, 100, 100));
		 auxEsf15->setPos(Vector3D(240, 100, 100));
		 auxCubo9->setPos(Vector3D(-100, 100, -100));
		 auxCubo10->setPos(Vector3D(-100, 100, -100));
		 auxCubo11->setPos(Vector3D(-100, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-200, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-220, 100, -100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));
	 }
	 if (e.getNivel() == 3) {
		 auxEsf8->setColMask(1, 1, 1, 2);
		 auxEsf8->setCol(Vector3D(1, 1, 0));
		 auxEsf9->setColMask(1, 1, 1, 2);
		 auxEsf9->setCol(Vector3D(1, 1, 0));
		 auxEsf10->setColMask(1, 1, 1, 2);
		 auxEsf10->setCol(Vector3D(1, 1, 0));

		 auxEsf8->setPos(Vector3D(0, 2, 0));
		 auxEsf8->setFija(true);

		 auxEsf9->setPos(Vector3D(-15, 4, 15));
		 auxEsf9->setVel(Vector3D(1, 0, -0.8));
		 auxEsf9->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf10->setPos(Vector3D(5, 8, 5));
		 auxEsf10->setVel(Vector3D(-0.6, 0, 0.74));
		 auxEsf10->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo2->setPos(Vector3D(-4, 6, 5));
		 auxCubo2->setVel(Vector3D(2, 0, -0.7));
		 auxCubo2->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo3->setPos(Vector3D(10, 6, -5));
		 auxCubo3->setVel(Vector3D(0.6, 0, -1.2));
		 auxCubo3->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo4->setPos(Vector3D(-5, 6, -15));
		 auxCubo4->setVel(Vector3D(-1, 0, 2));
		 auxCubo4->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf1->setPos(Vector3D(120, 100, 100));
		 auxEsf2->setPos(Vector3D(110, 100, 100));
		 auxEsf3->setPos(Vector3D(100, 100, 100));

		 auxEsf4->setPos(Vector3D(130, 100, 100));
		 auxEsf5->setPos(Vector3D(140, 100, 100));
		 auxEsf6->setPos(Vector3D(150, 100, 100));
		 auxEsf7->setPos(Vector3D(160, 100, 100));
		 auxCubo1->setPos(Vector3D(-100, 100, -100));

		 auxEsf11->setPos(Vector3D(200, 100, 100));
		 auxCubo5->setPos(Vector3D(-100, 100, -100));
		 auxCubo6->setPos(Vector3D(-100, 100, -100));
		 auxCubo7->setPos(Vector3D(-100, 100, -100));
		 auxCubo8->setPos(Vector3D(-100, 100, -100));

		 auxEsf12->setPos(Vector3D(210, 100, 100));
		 auxEsf13->setPos(Vector3D(220, 100, 100));
		 auxEsf14->setPos(Vector3D(230, 100, 100));
		 auxEsf15->setPos(Vector3D(240, 100, 100));
		 auxCubo9->setPos(Vector3D(-100, 100, -100));
		 auxCubo10->setPos(Vector3D(-100, 100, -100));
		 auxCubo11->setPos(Vector3D(-100, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-200, 100, -100));
		 auxCuboExtra1->setPos(Vector3D(-220, 100, -100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));
	 }
	 if (e.getNivel() == 4) {
		 auxEsf11->setColMask(1, 1, 1, 3);
		 auxEsf11->setCol(Vector3D(0, 0, 1));

		 auxEsf11->setPos(Vector3D(0, 3, -5));
		 auxEsf11->acumulaFuerza((Vector3D(0, -1, 0)));

		 auxCubo5->setPos(Vector3D(-20, 1.5, 0));
		 auxCubo5->setVel(Vector3D(7, 0, 0));

		 auxCubo6->setPos(Vector3D(20, 6, 0));
		 auxCubo6->setVel(Vector3D(-7, 0, 0));

		 auxCubo7->setPos(Vector3D(-20, 1.5, 5));
		 auxCubo7->setVel(Vector3D(7, 0, 0));

		 auxCubo8->setPos(Vector3D(20, 6, 5));
		 auxCubo8->setVel(Vector3D(-7, 0, 0));

		 auxCuboExtra1->setPos(Vector3D(-15, 4, 20));
		 auxCuboExtra2->setPos(Vector3D(15, 4, 20));
		 auxCuboExtra1->acumulaFuerza((Vector3D(0, -3, 0)));
		 auxCuboExtra1->setVel(Vector3D(0, 0, -4));
		 auxCuboExtra2->setVel(Vector3D(0, 0, -4));
		 auxCuboExtra2->acumulaFuerza((Vector3D(0, -3, 0)));

		 auxEsf1->setPos(Vector3D(120, 100, 100));
		 auxEsf2->setPos(Vector3D(110, 100, 100));
		 auxEsf3->setPos(Vector3D(100, 100, 100));

		 auxEsf4->setPos(Vector3D(130, 100, 100));
		 auxEsf5->setPos(Vector3D(140, 100, 100));
		 auxEsf6->setPos(Vector3D(150, 100, 100));
		 auxEsf7->setPos(Vector3D(160, 100, 100));
		 auxCubo1->setPos(Vector3D(-100, 100, -100));

		 auxEsf8->setPos(Vector3D(170, 100, 100));
		 auxEsf9->setPos(Vector3D(180, 100, 100));
		 auxEsf10->setPos(Vector3D(190, 100, 100));
		 auxCubo2->setPos(Vector3D(-110, 100, -100));
		 auxCubo3->setPos(Vector3D(-120, 100, -100));
		 auxCubo4->setPos(Vector3D(-130, 100, -100));

		 auxEsf12->setPos(Vector3D(210, 100, 100));
		 auxEsf13->setPos(Vector3D(220, 100, 100));
		 auxEsf14->setPos(Vector3D(230, 100, 100));
		 auxEsf15->setPos(Vector3D(240, 100, 100));
		 auxCubo9->setPos(Vector3D(-100, 100, -100));
		 auxCubo10->setPos(Vector3D(-100, 100, -100));
		 auxCubo11->setPos(Vector3D(-100, 100, -100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));
	 }
	 if (e.getNivel() == 5) {
		 auxEsf12->setColMask(1, 1, 1, 3);
		 auxEsf12->setCol(Vector3D(0, 0, 1));
		 auxEsf13->setColMask(1, 1, 1, 2);
		 auxEsf13->setCol(Vector3D(1, 1, 0));
		 auxEsf14->setColMask(1, 1, 1, 2);
		 auxEsf14->setCol(Vector3D(1, 1, 0));
		 auxEsf15->setColMask(1, 1, 1, 2);
		 auxEsf15->setCol(Vector3D(1, 1, 0));

		 auxEsf12->setPos(Vector3D(15, 5, 20));
		 auxEsf12->setFija(true);

		 auxEsf13->setPos(Vector3D(-5, 5, 0.3));
		 auxEsf13->setVel(Vector3D(1, 0, 0));
		 auxEsf13->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf14->setPos(Vector3D(5, 5, -0.3));
		 auxEsf14->setVel(Vector3D(-1, 0, 0));
		 auxEsf14->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf15->setPos(Vector3D(-7, 7, -5));
		 auxEsf15->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo9->setPos(Vector3D(-4, 6, 15));
		 auxCubo9->setVel(Vector3D(1, 0, -0.7));
		 auxCubo9->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo10->setPos(Vector3D(10, 6, -5));
		 auxCubo10->setVel(Vector3D(0.6, 0, -0.2));
		 auxCubo10->acumulaFuerza(Vector3D(0, -3, 0));

		 auxCubo11->setPos(Vector3D(0, 6, -15));
		 auxCubo11->setVel(Vector3D(0, 0, 1));
		 auxCubo11->acumulaFuerza(Vector3D(0, -3, 0));

		 auxEsf11->setPos(Vector3D(200, 100, 100));

		 mainChar->setPos(Vector3D(0, 5, 14));
		 mainChar->setVel(Vector3D(0, 0, 0));

	 }
	 e.setCambio(false);
	 cout << "Puntos: " << e.getPuntosN(e.getNivel() - 1) << endl;
 }

 //chequeadores secundarios que comprueban las puntuaciones de cada nivel y realizan el cambio si se cumple
 if (e.getNivel() == 1 && e.getPuntosN(1) > 25) {
	 e.setCambio(true);
 }
 if (e.getNivel() == 2 && e.getPuntosN(2) > 35) {
	 e.setCambio(true);
 }
 if (e.getNivel() == 3 && e.getPuntosN(3) > 25) {
	 e.setCambio(true);
 }
 if (e.getNivel() == 4 && e.getPuntosN(4) > 5) {
	 e.setCambio(true);
 }
 if (e.getNivel() == 5 && e.getPuntosN(5) > 75) {
	 e.setNivel(-1);
	 e.setCambio(true);
 }

}

//funcion para quardar puntuacion y nivel
void saveData() {
	ofstream myfile("save.txt");
	if (myfile.is_open()) {
		for (int i = 0; i < 5; i++) {
			myfile << e.getPuntosN(i) << endl;
		}
		myfile << e.getNivel() << endl;
	}
	myfile.close();
}

//funcion para quardar puntuacion y nivel con detalle
void saveDataPretty() {
	ofstream myfile("partidaGuardada.txt");
	if (myfile.is_open()) {
		myfile << "Archivo de datos de Juego" << endl;
		for (int i = 0; i < 5; i++) {
			myfile << "Puntos nivel: " << i << " = " << e.getPuntosN(i) << endl;
		}
		myfile << "Nivel guardado  :"<< e.getNivel() << endl;
	}
	myfile.close();
}

//funcion para cargar puntuacion y nivel, imprime "no eoncotrado" si no hay archivo save.txt
void loadData() {
	string line;
	ifstream myfile("save.txt");
	int count = 0;
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			if (count <= 4) {
				e.setPuntosN(count, atoi(line.c_str()));
			}
			if (count == 5) {
				e.setNivel(atoi(line.c_str()));
				e.setNivel(e.getNivel() - 1);
			}
			count++;
		}
		myfile.close();
		e.setCambio(true);
	}

	else cout << "Unable to open file";
}

//key pressed
void keyPressed(unsigned char key,int x,int y){
	Solido* s;
 switch(key){
	 //uso del array asci con booleanos para el movimiento
 case 's':
 case 'S':
	 asciKeyArr['w'] = true;
 break;
 case 'w':
 case 'W':
	 asciKeyArr['s'] = true;
 break;
 case 'a':
 case 'A':
	 asciKeyArr['a'] = true;
	 break;
 case 'd':
 case 'D':
	 asciKeyArr['d'] = true;
	 break;
 case ' ':
	 //salto
	 if (mainChar->getPos().getY() < 1.1) {
		 mainChar->setVel(Vector3D(0, 5.5, 0));
	 }
	 //si se esta en la pantalla de inicio empezar
	 if (e.getNivel() == 0) {
		 e.setCambio(true);
	 }
 break;
 case 'g':
 case 'G':
	 //guardar datos con funcion saveDAta() (arriba)
	 saveData();
	 saveDataPretty();
 break;
 case 'h':
 case 'H':
	 //cargar datos con funcion loadData() (arriba)
	 loadData();
	 break;
 case 'n':
 case 'N':
	 e.setCambio(true);
	 break;
 case 27:
   exit(0);
 break;
 }
}

//uso de keyUnpressed tambien para resetear los valores del array de booleanos usado para el movimiento
void keyUnpressed(unsigned char key, int x, int y){
	switch (key) {
	case 's':
	case 'S':
		asciKeyArr['w'] = false;
		break;
	case 'w':
	case 'W':
		asciKeyArr['s'] = false;
		break;
	case 'a':
	case 'A':
		asciKeyArr['a'] = false;
		break;
	case 'd':
	case 'D':
		asciKeyArr['d'] = false;
		break;
	}
}

//funcion que rota la camara si el raton esta presionado
void mouseMoved(int x, int y)
{
    if (mx>=0 && my>=0) {
    	for(unsigned int i=0;i<vistas.size();i++){
    		if(vistas[i].contain(x,y)){
    			Vector3D r;
    			Camara &cam=camaras[i];
    			r=cam.getRot()+Vector3D(y-my,x-mx,0);
    			cam.setRot(r);
    		}
    	}
    }
    mx = x;
    my = y;
}

void mousePress(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
    }
    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        mx = -1;
        my = -1;
    }
}

//inicializacion de todo
void init(void){
 glEnable(GL_DEPTH_TEST);
 glEnable(GL_LIGHTING);
 glEnable(GL_LIGHT0);
 glEnable(GL_LIGHT1);
 glEnable(GL_COLOR_MATERIAL);

 //inicializacion de algunas texturas
 ladrillos.init();
 paredTex.init();
 texTablero.init();

 testTexture.init();
 testTexture.setImage(imread("checker.jpg"));
 

 //inicializacion camara minimapa
 mapp = new CamaraFly();
 mapp->setPos(Vector3D(0, 45, -5));
 mapp->setRot(Vector3D(90, 0, 0));

}

//reshape
void reshape(int width,int height){
	for(Vista &v:vistas)
		v.reshape(width,height);
}




//main de la practica
int main(int argc, char** argv){
 srand(10);
 vel=0;

 //inicializacion de los puntos, nivel y booleano de cambio de nivel de escena e
 e.iniPuntos();
 e.setNivel(0);
 e.setCambio(false);

 //poscionamiento de camara TPS
 for(Camara &c:camaras)
	 c.setPos(Vector3D(0,1.65,10));
 
 


 //matrices tensoras de inercia y sus inversas
 double sphereIMT[9] = { 2.0 / 5.0, 0.0, 0.0, 0.0, 2.0 / 5.0 ,0.0 ,0.0 ,0.0 ,2.0 / 5.0 }; // asumiendo masa 1 radio 1
 matrixInvr(sphereIMT);
 double sphereIMT2[9] = { 2.0 / 5.0 * 4, 0.0, 0.0, 0.0, 2.0 / 5.0 * 4 ,0.0 ,0.0 ,0.0 ,2.0 / 5.0 * 4 };// asumiendo masa 1 radio 2
 matrixInvr(sphereIMT2);
 double sphereIMT05[9] = { 2.0 / 5.0 * 0.25, 0.0, 0.0, 0.0, 2.0 / 5.0 * 0.25,0.0 ,0.0 ,0.0 ,2.0 / 5.0 * 0.25 };// asumiendo masa 1 radio 0.5
 matrixInvr(sphereIMT05);
 double cubeIMT[9] = { 2.0 / 3.0, -1.0 / 4.0,  -1.0 / 4.0,  -1.0 / 4.0,  2.0 / 3.0,  -1.0 / 4.0,  -1.0 / 4.0,  -1.0 / 4.0,  2.0 / 3.0 }; // asumiendo masa 1 s 1
 matrixInvr(cubeIMT);
 double cubeIMT05[9] = { 2.0 / 3.0 * 0.25, -1.0 / 4.0* 0.25,  -1.0 / 4.0* 0.25,  -1.0 / 4.0* 0.25,  2.0 / 3.0* 0.25,  -1.0 / 4.0* 0.25,  -1.0 / 4.0* 0.25,  -1.0 / 4.0* 0.25,  2.0 / 3.0* 0.25 }; // asumiendo masa 1 s 0.5
 matrixInvr(cubeIMT05);
 double cubeIMT2[9] = { 2.0 / 3.0 * 8, -1.0 / 4.0* 8,  -1.0 / 4.0* 8,  -1.0 / 4.0* 8,  2.0 / 3.0*8,  -1.0 / 4.0* 8,  -1.0 / 4.0* 8,  -1.0 / 4.0* 8,  2.0 / 3.0*8 }; // asumiendo masa 1 s 0.5
 matrixInvr(cubeIMT05);
 

 //creacion objetos universales
 cuboNeutral1 = new Cubo();
 cuboNeutral1->setPos(Vector3D(-15, 1, -3));
 cuboNeutral1->setRot(Quaternion(3.14159 / 4, Vector3D(1, 0, 0)));
 cuboNeutral1->setRot(Quaternion(3.14159 / 4, Vector3D(0, 1, 0)));
 cuboNeutral1->setFija(true);
 cuboNeutral1->setCol(Vector3D(1, 1, 1));
 cuboNeutral1->setS(2);
 cuboNeutral1->setM(1);
 cuboNeutral1->setIMTInv(cubeIMT2[0], cubeIMT2[1], cubeIMT2[2], cubeIMT2[3], cubeIMT2[4], cubeIMT2[5], cubeIMT2[6], cubeIMT2[7], cubeIMT2[8]);
 cuboNeutral1->setElasR(0.7);
 cuboNeutral1->setColMask(1, 2, 2, 0);
 e.add(cuboNeutral1);

 //creacion objetos universales
 cuboNeutral2 = new Cubo();
 cuboNeutral2->setPos(Vector3D(15, 1, -3));
 cuboNeutral2->setRot(Quaternion(3.14159 / 4, Vector3D(1, 0, 0)));
 cuboNeutral2->setRot(Quaternion(3.14159 / 4, Vector3D(0, 1, 0)));
 cuboNeutral2->setFija(true);
 cuboNeutral2->setCol(Vector3D(1, 1, 1));
 cuboNeutral2->setS(2);
 cuboNeutral2->setM(1);
 cuboNeutral2->setIMTInv(cubeIMT2[0], cubeIMT2[1], cubeIMT2[2], cubeIMT2[3], cubeIMT2[4], cubeIMT2[5], cubeIMT2[6], cubeIMT2[7], cubeIMT2[8]);
 cuboNeutral2->setElasR(0.7);
 cuboNeutral2->setColMask(1, 2, 2, 0);
 e.add(cuboNeutral2);

 //creacion objetos universales
 cuboNeutral3 = new Cubo();
 cuboNeutral3->setPos(Vector3D(-10, 1, -22));
 cuboNeutral3->setFija(true);
 cuboNeutral3->setCol(Vector3D(1, 1, 1));
 cuboNeutral3->setS(2);
 cuboNeutral3->setM(1);
 cuboNeutral3->setIMTInv(cubeIMT2[0], cubeIMT2[1], cubeIMT2[2], cubeIMT2[3], cubeIMT2[4], cubeIMT2[5], cubeIMT2[6], cubeIMT2[7], cubeIMT2[8]);
 cuboNeutral3->setElasR(0.7);
 cuboNeutral3->setColMask(1, 2, 2, 0);
 e.add(cuboNeutral3);

 //creacion objetos universales
 cuboNeutral4 = new Cubo();
 cuboNeutral4->setPos(Vector3D(10, 1, -22));
 cuboNeutral4->setFija(true);
 cuboNeutral4->setCol(Vector3D(1, 1, 1));
 cuboNeutral4->setS(2);
 cuboNeutral4->setM(1);
 cuboNeutral4->setIMTInv(cubeIMT2[0], cubeIMT2[1], cubeIMT2[2], cubeIMT2[3], cubeIMT2[4], cubeIMT2[5], cubeIMT2[6], cubeIMT2[7], cubeIMT2[8]);
 cuboNeutral4->setElasR(0.7);
 cuboNeutral4->setColMask(1, 2, 2, 0);
 e.add(cuboNeutral4);




 //creacion de todos los objetos en todos los niveles (explicacion de que son IMT, elasR y maskCol en pdf)
 //se adjunta la matriz inversa de tensor de inercia correspondiente, su mascara de colision y su coeficiente elastico

 //NIVEL 1
 auxEsf1 = new Esfera();
 auxEsf1->setPos(Vector3D(120, 100, 100));
 auxEsf1->setR(1);
 auxEsf1->setM(1);
 auxEsf1->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf1->setElasR(0.9);
 auxEsf1->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf1->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf1);

 auxEsf2 = new Esfera();
 auxEsf2->setPos(Vector3D(110, 100, 100));
 auxEsf2->setR(1);
 auxEsf2->setM(1);
 auxEsf2->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf2->setElasR(0.9);
 auxEsf2->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf2->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf2);

 auxEsf3 = new Esfera();
 auxEsf3->setPos(Vector3D(100, 100, 100));
 auxEsf3->setR(1);
 auxEsf3->setM(1);
 auxEsf3->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf3->setElasR(0.9);
 auxEsf3->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf3->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf3);

 //NIVEL 2

 auxCubo1 = new Cubo();
 auxCubo1->setPos(Vector3D(-100, 100, -100));
 auxCubo1->setCol(Vector3D(1, 0, 0));
 auxCubo1->setS(1);
 auxCubo1->setM(1);
 auxCubo1->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo1->setElasR(0.9);
 auxCubo1->setColMask(1, 2, 1, 4);
 e.add(auxCubo1);

 auxEsf4 = new Esfera();
 auxEsf4->setPos(Vector3D(130, 100, 100));
 auxEsf4->setR(1);
 auxEsf4->setM(1);
 auxEsf4->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf4->setElasR(0.9);
 auxEsf4->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf4->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf4);

 auxEsf5 = new Esfera();
 auxEsf5->setPos(Vector3D(140, 100, 100));
 auxEsf5->setR(1);
 auxEsf5->setM(1);
 auxEsf5->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf5->setElasR(0.9);
 auxEsf5->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf5->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf5);

 auxEsf6 = new Esfera();
 auxEsf6->setPos(Vector3D(150, 100, 100));
 auxEsf6->setR(0.5);
 auxEsf6->setM(1);
 auxEsf6->setIMTInv(sphereIMT05[0], sphereIMT05[1], sphereIMT05[2], sphereIMT05[3], sphereIMT05[4], sphereIMT05[5], sphereIMT05[6], sphereIMT05[7], sphereIMT05[8]);
 auxEsf6->setElasR(0.9);
 auxEsf6->setColMask(1, 1, 0.5, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf6->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf6);

 auxEsf7 = new Esfera();
 auxEsf7->setPos(Vector3D(160, 100, 100));
 auxEsf7->setR(0.5);
 auxEsf7->setM(1);
 auxEsf7->setIMTInv(sphereIMT05[0], sphereIMT05[1], sphereIMT05[2], sphereIMT05[3], sphereIMT05[4], sphereIMT05[5], sphereIMT05[6], sphereIMT05[7], sphereIMT05[8]);
 auxEsf7->setElasR(0.9);
 auxEsf7->setColMask(1, 1, 0.5, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf7->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf7);

 

 //NIVEL 3

 auxCubo2 = new Cubo();
 auxCubo2->setPos(Vector3D(-110, 100, -100));
 auxCubo2->setCol(Vector3D(1, 0, 0));
 auxCubo2->setS(0.5);
 auxCubo2->setM(1);
 auxCubo2->setIMTInv(cubeIMT05[0], cubeIMT05[1], cubeIMT05[2], cubeIMT05[3], cubeIMT05[4], cubeIMT05[5], cubeIMT05[6], cubeIMT05[7], cubeIMT05[8]);
 auxCubo2->setElasR(0.9);
 auxCubo2->setColMask(1, 2, 1, 4);
 e.add(auxCubo2);

 auxCubo3 = new Cubo();
 auxCubo3->setPos(Vector3D(-120, 100, -100));
 auxCubo3->setCol(Vector3D(1, 0, 0));
 auxCubo3->setS(0.5);
 auxCubo3->setM(1);
 auxCubo3->setIMTInv(cubeIMT05[0], cubeIMT05[1], cubeIMT05[2], cubeIMT05[3], cubeIMT05[4], cubeIMT05[5], cubeIMT05[6], cubeIMT05[7], cubeIMT05[8]);
 auxCubo3->setElasR(0.9);
 auxCubo3->setColMask(1, 2, 1, 4);
 e.add(auxCubo3);

 auxCubo4 = new Cubo();
 auxCubo4->setPos(Vector3D(-130, 100, -100));
 auxCubo4->setCol(Vector3D(1, 0, 0));
 auxCubo4->setS(0.5);
 auxCubo4->setM(1);
 auxCubo4->setIMTInv(cubeIMT05[0], cubeIMT05[1], cubeIMT05[2], cubeIMT05[3], cubeIMT05[4], cubeIMT05[5], cubeIMT05[6], cubeIMT05[7], cubeIMT05[8]);
 auxCubo4->setElasR(0.9);
 auxCubo4->setColMask(1, 2, 1, 4);
 e.add(auxCubo4);


 auxEsf8 = new Esfera();
 auxEsf8->setPos(Vector3D(170, 100, 100));
 auxEsf8->setR(2);
 auxEsf8->setM(1);
 auxEsf8->setIMTInv(sphereIMT2[0], sphereIMT2[1], sphereIMT2[2], sphereIMT2[3], sphereIMT2[4], sphereIMT2[5], sphereIMT2[6], sphereIMT2[7], sphereIMT2[8]);
 auxEsf8->setElasR(0.9);
 auxEsf8->setColMask(1, 1, 2, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf8->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf8);

 auxEsf9 = new Esfera();
 auxEsf9->setPos(Vector3D(180, 100, 100));
 auxEsf9->setR(1);
 auxEsf9->setM(1);
 auxEsf9->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf9->setElasR(0.9);
 auxEsf9->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf9->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf9);

 auxEsf10 = new Esfera();
 auxEsf10->setPos(Vector3D(190, 100, 100));
 auxEsf10->setR(1);
 auxEsf10->setM(1);
 auxEsf10->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf10->setElasR(0.9);
 auxEsf10->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf10->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf10);

 

 //NIVEL 4

 auxCubo5 = new Cubo();
 auxCubo5->setPos(Vector3D(-100, 100, -100));
 auxCubo5->setCol(Vector3D(1, 0, 0));
 auxCubo5->setS(1);
 auxCubo5->setM(1);
 auxCubo5->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo5->setElasR(0.9);
 auxCubo5->setColMask(1, 2, 1, 4);
 e.add(auxCubo5);

 auxCubo6 = new Cubo();
 auxCubo6->setPos(Vector3D(-100, 100, -100));
 auxCubo6->setCol(Vector3D(1, 0, 0));
 auxCubo6->setS(1);
 auxCubo6->setM(1);
 auxCubo6->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo6->setElasR(0.9);
 auxCubo6->setColMask(1, 2, 1, 4);
 e.add(auxCubo6);

 auxCubo7 = new Cubo();
 auxCubo7->setPos(Vector3D(-100, 100, -100));
 auxCubo7->setCol(Vector3D(1, 0, 0));
 auxCubo7->setS(1);
 auxCubo7->setM(1);
 auxCubo7->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo7->setElasR(0.9);
 auxCubo7->setColMask(1, 2, 1, 4);
 e.add(auxCubo7);

 auxCubo8 = new Cubo();
 auxCubo8->setPos(Vector3D(-100, 100, -100));
 auxCubo8->setCol(Vector3D(1, 0, 0));
 auxCubo8->setS(1);
 auxCubo8->setM(1);
 auxCubo8->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo8->setElasR(0.9);
 auxCubo8->setColMask(1, 2, 1, 4);
 e.add(auxCubo8);

 auxCuboExtra1 = new Cubo();
 auxCuboExtra1->setPos(Vector3D(-100, 100, -100));
 auxCuboExtra1->setCol(Vector3D(1, 0, 0));
 auxCuboExtra1->setS(1);
 auxCuboExtra1->setM(1);
 auxCuboExtra1->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCuboExtra1->setElasR(0.9);
 auxCuboExtra1->setColMask(1, 2, 1, 4);
 e.add(auxCuboExtra1);

 auxCuboExtra2 = new Cubo();
 auxCuboExtra2->setPos(Vector3D(-100, 100, -100));
 auxCuboExtra2->setCol(Vector3D(1, 0, 0));
 auxCuboExtra2->setS(1);
 auxCuboExtra2->setM(1);
 auxCuboExtra2->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCuboExtra2->setElasR(0.9);
 auxCuboExtra2->setColMask(1, 2, 1, 4);
 e.add(auxCuboExtra2);

 auxEsf11 = new Esfera();
 auxEsf11->setPos(Vector3D(200, 100, 100));
 auxEsf11->setR(1);
 auxEsf11->setM(1);
 auxEsf11->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf11->setElasR(0.9);
 auxEsf11->setColMask(1, 1, 1, 3); //esfera con radio 1 (resto de params inecesarios)
 auxEsf11->setCol(Vector3D(0, 0, 1));
 e.add(auxEsf11);


 

 //NIVEL 5

 auxCubo9 = new Cubo();
 auxCubo9->setPos(Vector3D(-100, 100, -100));
 auxCubo9->setCol(Vector3D(1, 0, 0));
 auxCubo9->setS(1);
 auxCubo9->setM(1);
 auxCubo9->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo9->setElasR(0.9);
 auxCubo9->setColMask(1, 2, 1, 4);
 e.add(auxCubo9);

 auxCubo10 = new Cubo();
 auxCubo10->setPos(Vector3D(-100, 100, -100));
 auxCubo10->setCol(Vector3D(1, 0, 0));
 auxCubo10->setS(1);
 auxCubo10->setM(1);
 auxCubo10->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo10->setElasR(0.9);
 auxCubo10->setColMask(1, 2, 1, 4);
 e.add(auxCubo10);

 auxCubo11 = new Cubo();
 auxCubo11->setPos(Vector3D(-100, 100, -100));
 auxCubo11->setCol(Vector3D(1, 0, 0));
 auxCubo11->setS(1);
 auxCubo11->setM(1);
 auxCubo11->setIMTInv(cubeIMT[0], cubeIMT[1], cubeIMT[2], cubeIMT[3], cubeIMT[4], cubeIMT[5], cubeIMT[6], cubeIMT[7], cubeIMT[8]);
 auxCubo11->setElasR(0.9);
 auxCubo11->setColMask(1, 2, 1, 4);
 e.add(auxCubo11);

 auxEsf12 = new Esfera();
 auxEsf12->setPos(Vector3D(210, 100, 100));
 auxEsf12->setR(2);
 auxEsf12->setM(1);
 auxEsf12->setIMTInv(sphereIMT2[0], sphereIMT2[1], sphereIMT2[2], sphereIMT2[3], sphereIMT2[4], sphereIMT2[5], sphereIMT2[6], sphereIMT2[7], sphereIMT2[8]);
 auxEsf12->setElasR(0.9);
 auxEsf12->setColMask(1, 1, 2, 3); //esfera con radio 1 (resto de params inecesarios)
 auxEsf12->setCol(Vector3D(0, 0, 1));
 e.add(auxEsf12);

 auxEsf13 = new Esfera();
 auxEsf13->setPos(Vector3D(220, 100, 100));
 auxEsf13->setR(1);
 auxEsf13->setM(1);
 auxEsf13->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf13->setElasR(0.9);
 auxEsf13->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf13->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf13);

 auxEsf14 = new Esfera();
 auxEsf14->setPos(Vector3D(230, 100, 100));
 auxEsf14->setR(1);
 auxEsf14->setM(1);
 auxEsf14->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf14->setElasR(0.9);
 auxEsf14->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf14->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf14);

 auxEsf15 = new Esfera();
 auxEsf15->setPos(Vector3D(240, 100, 100));
 auxEsf15->setR(1);
 auxEsf15->setM(1);
 auxEsf15->setIMTInv(sphereIMT[0], sphereIMT[1], sphereIMT[2], sphereIMT[3], sphereIMT[4], sphereIMT[5], sphereIMT[6], sphereIMT[7], sphereIMT[8]);
 auxEsf15->setElasR(0.9);
 auxEsf15->setColMask(1, 1, 1, 2); //esfera con radio 1 (resto de params inecesarios)
 auxEsf15->setCol(Vector3D(1, 1, 0));
 e.add(auxEsf15);

 


 //creacion de personaje principal (explicacion de que son IMT, elasR y maskCol en pdf)
 //se adjunta la matriz inversa de tensor de inercia correspondiente, su mascara de colision y su coeficiente elastico
 mainChar = new Esfera();
 mainChar->setR(1);
 mainChar->setCol(Vector3D(0, 1, 1));
 camaras[0].setSolido(mainChar);
 mainChar->setM(1);
 mainChar->acumulaFuerza(Vector3D(0, -3, 0));
 mainChar->setPos(Vector3D(0, 5, 14));
 double mainSphereIMT[9] = { (2.0 / 5.0) * mainChar->getM(), 0.0, 0.0, 0.0, (2.0 / 5.0) * mainChar->getM() ,0.0 ,0.0 ,0.0 ,(2.0 / 5.0) * mainChar->getM() };
 matrixInvr(mainSphereIMT);
 mainChar->setIMTInv(mainSphereIMT[0], mainSphereIMT[1], mainSphereIMT[2], mainSphereIMT[3], mainSphereIMT[4], mainSphereIMT[5], mainSphereIMT[6], mainSphereIMT[7], mainSphereIMT[8]);
 mainChar->setElasR(1);
 mainChar->setColMask(1, 1, 1, 1);
 e.add(mainChar);


 //paredes limite
 double fieldleng = 25;
 double fieldHeight = 5;
 Vector3D pp0(-fieldleng, 0.5, fieldleng);
 Vector3D pp1(-fieldleng, 0, fieldleng);
 Vector3D pp2(fieldleng, 0, fieldleng);
 Vector3D pp3(fieldleng, 0.5, fieldleng);
 Rectangulo* ret = new Rectangulo(pp0, pp1, pp2, pp3);
 ret->setCol(Vector3D(0.5, 0.5, 0.5));
 ret->iniRendT();
 ret->setCol(Vector3D(1, 1, 1));
 //ladrillos.setImage(imread("mario_kart_circuit.jpg"));
 //ladrillos.setImage(imread("paredTextura.jpg"));
 //ret->getTex() = ladrillos;
 ret->setNU(1);
 ret->setNV(1);
 e.add(ret);

 pp0 = Vector3D(-fieldleng, fieldHeight, -fieldleng);
 pp1 = Vector3D(-fieldleng, 0, -fieldleng);
 pp2 = Vector3D(fieldleng, 0, -fieldleng);
 pp3 = Vector3D(fieldleng, fieldHeight, -fieldleng);
 ret = new Rectangulo(pp0, pp1, pp2, pp3);
 ret->setCol(Vector3D(0.5, 0.5, 0.5));
 ret->iniRendT();
 ret->setCol(Vector3D(1, 1, 1));
 //ladrillos.setImage(imread("mario_kart_circuit.jpg"));
 //ladrillos.setImage(imread("paredTextura.jpg"));
 //ret->getTex() = ladrillos;
 ret->setNU(1);
 ret->setNV(1);
 e.add(ret);

 pp0 = Vector3D(fieldleng, fieldHeight, -fieldleng);
 pp1 = Vector3D(fieldleng, 0, -fieldleng);
 pp2 = Vector3D(fieldleng, 0, fieldleng);
 pp3 = Vector3D(fieldleng, fieldHeight, fieldleng);
 ret = new Rectangulo(pp0, pp1, pp2, pp3);
 ret->setCol(Vector3D(0.5, 0.5, 0.5));
 ret->iniRendT();
 ret->setCol(Vector3D(1, 1, 1));
 //ladrillos.setImage(imread("mario_kart_circuit.jpg"));
 //ladrillos.setImage(imread("paredTextura.jpg"));
// ret->getTex() = ladrillos;
 ret->setNU(1);
 ret->setNV(1);
 e.add(ret);

 pp0 = Vector3D(-fieldleng, fieldHeight, -fieldleng);
 pp1 = Vector3D(-fieldleng, 0, -fieldleng);
 pp2 = Vector3D(-fieldleng, 0, fieldleng);
 pp3 = Vector3D(-fieldleng, fieldHeight, fieldleng);
 ret = new Rectangulo(pp0, pp1, pp2, pp3);
 ret->setCol(Vector3D(0.5, 0.5, 0.5));
 ret->iniRendT();
 ret->setCol(Vector3D(1, 1, 1));
 //ladrillos.setImage(imread("mario_kart_circuit.jpg"));
 //ladrillos.setImage(imread("paredTextura.jpg"));
 //ret->getTex() = ladrillos;
 ret->setNU(1);
 ret->setNV(1);
 e.add(ret);

 std::cout<<"Escena guardada"<<endl;
 //cout <<e0<<endl;

 //resto de funciones de glut y opengl
 glutInit(&argc,argv);
 //glutInitDisplayMode(GLUT_SINGLE);
 glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
 glutInitWindowSize(640*2,480);
 glutInitWindowPosition(300,300);
 glutCreateWindow("Cubes and Spheres");
 init();

 //suelo
 Vector3D p0(-25,0,-25);
 Vector3D p1(-25,0, 25);
 Vector3D p2( 25,0, 25);
 Vector3D p3( 25,0,-25);
 //Rectangulo *ret;
 ret=new Rectangulo(p0,p1,p2,p3);
 ret->setCol(Vector3D(1,1,1));
 //ladrillos.setImage(imread("mario_kart_circuit.jpg"));
 ladrillos.setImage(imread("sueloTextura.jpg"));
 ret->getTex()=ladrillos;
 ret->setNU(1);
 ret->setNV(1);
 e.add(ret);

 //mensaje de presionar espacio al empezar
 pared = new Rectangulo(Vector3D(25, 12, 25), Vector3D(25, 12, -25), Vector3D(-25, 12, -25), Vector3D(-25, 12, 25));
 ret->setCol(Vector3D(1,1,1));
 paredTex.setImage(imread("brown_brick_texture_map.jpg"));
 pared->getTex()=paredTex;
 pared->setNU(1);
 pared->setNV(1);
 e.add(pared);

 //textura de fondo
 fondo.setTextura(testTexture);
 fondoTablero.setTextura(texTablero);

 glutDisplayFunc(displayMe);
 glutIdleFunc(idle);
 glutReshapeFunc(reshape);
 glutIgnoreKeyRepeat(1); //ignorar multiples llamadas de la misma tecla (movimientos mas fluidos)
 glutKeyboardFunc(keyPressed);
 glutKeyboardUpFunc(keyUnpressed);
 glutMotionFunc(&mouseMoved);
 glutMouseFunc(&mousePress);
 glutMainLoop();
 return 0;
}


