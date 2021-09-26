#ifndef CUBO_H_
#define CUBO_H_
#include <GL/glut.h>
#include "vector3d.h"
#include "solido.h"
#include <vector>

class Cubo: public Solido{
 float s;
public:
 Cubo():s(1){}
 Cubo(const Cubo &c):Solido(c),s(c.s){}
 Cubo *clone(){
	 return new Cubo(*this);
 }
 inline float getS(){return s;}
 inline void setS(float sp){s=sp;}
 void render();
 vector<Vector3D> getNormales(){

	 vector<Vector3D> ret;
	 Quaternion normX(0, 1, 0, 0);
	 Quaternion normY(0, 0, 1, 0);
	 Quaternion normZ(0, 0, 0, 1);

	 normX = getRotq() * normX * getRotq().inverseRet();
	 Vector3D vectNormX = normX.toVect();
	 vectNormX.normalize();
	 ret.push_back(vectNormX);

	 normY = getRotq() * normY * getRotq().inverseRet();
	 Vector3D vectNormY = normY.toVect();
	 vectNormY.normalize();
	 ret.push_back(vectNormY);

	 normZ = getRotq() * normZ * getRotq().inverseRet();
	 Vector3D vectNormZ = normZ.toVect();
	 vectNormZ.normalize();
	 ret.push_back(vectNormZ);

	 return ret;
 }

 vector<Vector3D> getEsquinas(vector<Vector3D> normales){

	 vector<Vector3D> ret;

	 ret.push_back(getPos() + ((normales[0] + normales[1] + normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((normales[0] + normales[1] - normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((normales[0] - normales[1] + normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((normales[0] - normales[1] - normales[2]) * (s / 2)));

	 ret.push_back(getPos() + ((-normales[0] + normales[1] + normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((-normales[0] + normales[1] - normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((-normales[0] - normales[1] + normales[2]) * (s / 2)));
	 ret.push_back(getPos() + ((-normales[0] - normales[1] - normales[2]) * (s / 2)));

	 return ret;
 }
 friend std::ostream &operator << (std::ostream &os, const Cubo &v);
};
inline std::ostream &operator<<(std::ostream &os, const Cubo &c){
	os << static_cast<const Solido&>( c );
	os <<",S="<< c.s;
    return os;
}
#endif
