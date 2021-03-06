/*
 * esfera.cpp
 *
 *  Created on: 13 Oct 2016
 *      Author: Francisco Dominguez
 */
#include "esfera.h"
void Esfera::render(){
   glPushMatrix();
   glColor3f(this->getCol().getX(),this->getCol().getY(),this->getCol().getZ());
   glTranslatef(this->getPos().getX(),this->getPos().getY(),this->getPos().getZ());
   glMultMatrixd(this->getRotq().toMat4()); //rotatcion por quaterniones
   glutSolidSphere(r,10,10);
   glPopMatrix();
}





