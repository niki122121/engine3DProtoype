#pragma once

#include <iostream>
#include <cassert>
#include "vector3d.h"
#include <GL/glut.h>

using namespace std;

//plantilla Quaternion
template <class S> class Quaternionx;

typedef Quaternionx<double> Quaternion;

typedef Quaternionx<float>  Quaternionf;
typedef Quaternionx<double> Quaterniond;
typedef Quaternionx<int>    Quaternioni;

template <class S>
class Quaternionx {
	S r, i, j, k;
public:
	//distintas maneras de inicializarlo, pasandole 4 valores, un angulo y un eje o simplemente un vector
	Quaternionx() :Quaternionx(0, 0, 0, 0) {}
	Quaternionx(const S &r, const S &i, const S &j, const S &k) :r(r), i(i), j(j), k(k) {}
	Quaternionx(double angle, Vector3D axis) {
		r = cos(angle / 2);
		axis.normalize();
		i = sin(angle / 2)*axis.getX();
		j = sin(angle / 2)*axis.getY();
		k = sin(angle / 2)*axis.getZ();
	}
	Quaternionx(Vector3D toquat) {
		r = 0;
		i = toquat.getX();
		j = toquat.getY();
		k = toquat.getZ();
	}
	Quaternionx(const Quaternionx<S> &q) :r(q.r), i(q.i), j(q.j), k(q.k) {}
	Quaternionx<S> *clone() { return new Quaternionx<S>(*this); }

	//operadores comunes
	Quaternionx<S> operator+(const Quaternionx<S> &b) {
		Quaternionx<S> ret;
		ret.r = r + b.r;
		ret.i = i + b.i;
		ret.j = j + b.j;
		ret.k = k + b.k;
		return ret;
	}
	Quaternionx<S> operator-(const Quaternionx<S> &b) {
		Quaternionx<S> ret;
		ret.r = r - b.r;
		ret.i = i - b.i;
		ret.j = j - b.j;
		ret.k = k - b.k;
		return ret;
	}
	Quaternionx<S> operator*(const Quaternionx<S> &b) {
		Quaternionx<S> ret;
		ret.r = r * b.r - i * b.i - j * b.j - k * b.k;
		ret.i = r * b.i + i * b.r + j * b.k - k * b.j;
		ret.j = r * b.j - i * b.k + j * b.r + k * b.i;
		ret.k = r * b.k + i * b.j - j * b.i + k * b.r;
		return ret;
	}
	/*
	Quaternionx<S> quatXvectQ(const Quaternionx<S> &b) {
		Quaternionx<S> ret;
		ret.r = - i * b.i - j * b.j - k * b.k;
		ret.i = r * b.i + j * b.k - k * b.j;
		ret.j = r * b.j - i * b.k + k * b.i;
		ret.k = r * b.k + i * b.j - j * b.i;
		return ret;
	}

	Quaternionx<S> vectQXquat(const Quaternionx<S> &b) { 
		Quaternionx<S> ret;
		ret.r = - i * b.i - j * b.j - k * b.k;
		ret.i = i * b.r + j * b.k - k * b.j;
		ret.j = - i * b.k + j * b.r + k * b.i;
		ret.k = i * b.j - j * b.i + k * b.r;
		return ret;
	}*/

	Quaternionx<S> operator*(double d) {
		Quaternionx<S> ret;
		ret.r = r * d;
		ret.i = i * d;
		ret.j = j * d;
		ret.k = k * d;
		return ret;
	}

	//getters y setters
	inline S &getR() { return r; }
	inline S &getI() { return i; }
	inline S &getJ() { return j; }
	inline S &getK() { return k; }

	inline void setR(const double &pr) { x = pe; }
	inline void setI(const double &pi) { y = pi; }
	inline void setJ(const double &pj) { z = pj; }
	inline void setK(const double &pk) { z = pk; }

	//resto de operaciones como longitud, si es puramente 4d, normalizar, inverso... parecidas a Vector3D
	inline S length() {
		return sqrt(r*r + i * i + j * j + k * k);
	}

	inline bool isPure4D() {
		if (r == 0)
			return true;
		return false;
	}

	inline void normalize() {
		if ((r > 0.00001 || r < -0.00001) || (i > 0.00001 || i < -0.00001) || (j > 0.00001 || j < -0.00001) || (k > 0.00001 || k < -0.00001)) {
			S aux = length();

			if (aux == 0)
				throw runtime_error("Normal muy pequeña");

			aux = 1.0f / aux;

			r *= aux;
			i *= aux;
			j *= aux;
			k *= aux;
		}
		else {
			r = 1; i = 0; j = 0; k = 0;
		}
	}

	//un qua inverso = r,-i,-j,-k
	inline void inverse() {
		i = -i;
		k = -k;
		j = -j;
	}

	inline Quaternion inverseRet() {
		Quaternion ret(r, -i, -j, -k);
		return ret;
	}

	//parecido a inicializacion por angulo y eje, simplemente se rota el quaternion a la posicion deseada
	inline void rotate(double angle, Vector3D axis) {
		r = cos(angle / 2);
		axis.normalize();
		i = sin(angle / 2)*axis.getX();
		j = sin(angle / 2)*axis.getY();
		k = sin(angle / 2)*axis.getZ();
		cout << r << " " << i << " " << j << " " << k << endl;
	}

	//se convierte el Quaternion a matriz 4x4 y para que glut y opengl puedan trabajar con ellos
	inline GLdouble* toMat4() {
		GLdouble mat[16];

		double ii, ij, ik, ir, jj, jk, jr, kk, kr;

		ii = i * i;
		ij = i * j;
		ik = i * k;
		ir = i * r;

		jj = j * j;
		jk = j * k;
		jr = j * r;

		kk = k * k;
		kr = k * r;

		mat[0] = 1 - 2 * (jj + kk);
		mat[1] = 2 * (ij + kr);
		mat[2] = 2 * (ik - jr);
		mat[3] = 0.0;

		mat[4] = 2 * (ij - kr);
		mat[5] = 1 - 2 * (ii + kk);
		mat[6] = 2 * (jk + ir);
		mat[7] = 0.0;

		mat[8] = 2 * (ik + jr);
		mat[9] = 2 * (jk - ir);
		mat[10] = 1 - 2 * (ii + jj);
		mat[11] = 0.0;

		mat[12] = 0.0;
		mat[13] = 0.0;
		mat[14] = 0.0;
		mat[15] = 1.0;

		return mat;
	}

	inline Vector3D toVect() {
		Vector3D ret(i, j, k);
		return ret;
	}

};

