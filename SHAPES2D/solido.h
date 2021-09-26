/*
 * solido.h
 *
 *  Created on: 25 Oct 2016
 *      Author: francisco
 */

#ifndef SOLIDO_H_
#define SOLIDO_H_
#include <GL/glut.h>
#include "vector3d.h"
#include "quaternion.h"

//mismo codigo que solido normal pero incluyendo las variables elasR, IMT, colMask (explicadas detalladamente en el pdf)
class Solido {
	 Vector3D pos;
	 Quaterniond rot;
	 Vector3D vel;
	 Quaternion angVel;
	 Vector3D color;
	 Vector3D f;
	 double m;
	 bool fija;
	 double edad;

	 //adicionales para collisiones
	 double elasR;
	 double inertiaMatrixTensorInverse[9];
	 double colMask[4];


public:
	 Solido(Vector3D p):pos(p),rot(Quaterniond(1,0,0,0)),vel(0,0,0), angVel(Quaterniond(1, 0, 0, 0)),color(1,1,1),f(0,0,0),m(1.0),fija(false),edad(0),elasR(0.0){} //+adicionales
	 Solido(double x,double y,double z):Solido(Vector3D(x,y,z)){}
	 Solido():Solido(0,0,0) {}
	 Solido(const Solido &s):pos(s.pos),vel(s.vel),rot(Quaterniond(1,0,0,0)),angVel(Quaterniond(1, 0, 0, 0)),color(s.color),f(s.f),m(s.m),fija(false),elasR(s.elasR){}
	 virtual Solido *clone (){return new Solido(*this);}
	 inline Vector3D getPos(){return pos;}
	 inline Vector3D getVel(){return vel;}
	 inline Vector3D getCol(){return color;}
	 inline Vector3D getF(){return f;}
	 inline double getM(){return m;}
	 inline bool getFija(){return fija;}
	 inline Vector3D getRot() { return rot.toVect(); }
	 inline Quaterniond getRotq() { return rot; }
	 inline void setRot(Quaterniond &q) { rot = q; }
	 inline Quaterniond getAngVel() { return angVel; }
	 inline void setAngVel(Quaterniond &w) { angVel = w; }
	 inline void setPos(Vector3D p){pos=p;}
	 inline void setVel(Vector3D v){vel=v;}
	 inline void setCol(Vector3D c){color=c;}
	 inline virtual void setF(Vector3D v){f=v;}
	 inline void setM(double mp){m=mp;}
	 inline void setFija(bool b){fija=b;}
	 inline void hazFija (){fija=true;}
	 inline void hazMovil(){fija=false;}
	 inline void limpiaFuerza(){f=Vector3D();}
	 inline void aplicaFuerza (Vector3D fv){f=f+fv;}
	 inline void acumulaFuerza(Vector3D fv){f=f+fv;}

	 inline void rota(Quaterniond rq) {
		 rot = rot * rq;
		 rot.normalize();
	 }


	 inline double getElasR() { return elasR; }
	 inline void setElasR(double er) { 
		 if (er > 1.0)
			 er = 1.0;
		 if (er < 0.0)
			 er = 0.0;
		 elasR = er;
	 }

	 inline double *getIMTInv() { return inertiaMatrixTensorInverse; }
	 inline double getIMTInvNum(int num) { return inertiaMatrixTensorInverse[num]; }
	 inline void setIMTInv(double imt0, double imt1, double imt2, double imt3, double imt4, double imt5, double imt6, double imt7, double imt8 ) {
		 inertiaMatrixTensorInverse[0] = imt0; inertiaMatrixTensorInverse[1] = imt1; inertiaMatrixTensorInverse[2] = imt2; inertiaMatrixTensorInverse[3] = imt3; inertiaMatrixTensorInverse[4] = imt4; inertiaMatrixTensorInverse[5] = imt5; inertiaMatrixTensorInverse[6] = imt6; inertiaMatrixTensorInverse[7] = imt7; inertiaMatrixTensorInverse[8] = imt8;
	 }
	 inline double* getColMask() { return colMask; }
	 inline double getColMaskNum(int num) { return colMask[num]; }
	 inline void setColMask(double cM1, double cM2, double cM3, double cM4) { colMask[0] = cM1; colMask[1] = cM2; colMask[2] = cM3; colMask[3] = cM4;}

	 inline virtual void update(double dt) {
		 if (fija) {
			 vel = Vector3D();
			 angVel = Quaterniond();
			 return;
		 }
		 vel = vel + f / m * dt;
		 pos = pos + vel * dt;

		 //Cada update tambien tiene en cuenta la velocidad angular
		 rota(angVel);

		 double collisionExtra = 0;
		 double sizeField = 25;
		 if (getColMaskNum(1) == 1) {
			 collisionExtra = getColMaskNum(2);
		 }
		 else if (getColMaskNum(1) == 2){
			 collisionExtra = getColMaskNum(2)/2;
		 }

		 if(pos.getX() + collisionExtra > sizeField){
			 vel.setX(-abs(vel.getX()*0.75));
			 angVel = angVel.inverseRet() * 0.5;
		 }
		 if (pos.getX() - collisionExtra < -sizeField) {
			 vel.setX(abs(vel.getX()*0.75));
			 angVel = angVel.inverseRet() * 0.5;
		 }
		 if(pos.getZ() + collisionExtra > sizeField){
		 	 vel.setZ(-abs(vel.getZ()*0.75));
			 angVel = angVel.inverseRet() * 0.5;
		 }
		 if (pos.getZ() - collisionExtra < -sizeField) {
			 vel.setZ(abs(vel.getZ()*0.75));
			 angVel = angVel.inverseRet() * 0.5;
		 }
		 if(pos.getY()<collisionExtra-0.1){
			 vel.setY(vel.getY()*-0.99);
			 if (getColMaskNum(3) == 1) {
				 vel.setY(vel.getY()*0.6);
			 }
			 pos.setY(0.0001 + collisionExtra - 0.1);
			 angVel = angVel * 0.95;
		 }
	 }
	 virtual void render(){
		   glPushMatrix();
		   glColor3f(this->getCol().getX(),this->getCol().getY(),this->getCol().getZ());
		   glTranslatef(this->getPos().getX(),this->getPos().getY(),this->getPos().getZ());
		   glutSolidSphere(0.1,10,10);
		   glPopMatrix();
	 }
    friend std::ostream &operator << (std::ostream &os, const Solido &v);
};
inline std::ostream &operator<<(std::ostream &os, const Solido &s){
    os << "Pos=" << s.pos << ",Vel=" << s.vel << ",Col=" << s.color;
    return os;
}
#endif /* SOLIDO_H_ */
