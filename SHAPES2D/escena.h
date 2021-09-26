/*
 * escena.h
 *
 *  Created on: 18 Oct 2016
 *      Author: francisco
 */

#ifndef ESCENA_H_
#define ESCENA_H_
#include <vector>
#include "vector3d.h"
#include "solido.h"
#include "luz.h"
#include <iostream>
#include <math.h> 
#include <algorithm>
using namespace std;

class Escena {
	vector<Solido*> solidos;
	//params adicionales para el juego: puntuacion niveles...
	int nivel;
	int puntos[6];
	bool cambio;
public:
	Escena();
	Escena(const Escena &e): nivel(e.nivel){
		for(Solido *s:e.solidos){
			solidos.push_back(s->clone());
		}
	}
	virtual ~Escena();
	void render(){
		for(Solido *s:solidos)
			s->render();
	}
	void update(double dt){
		 for(Solido *s:solidos){
			 //s->setF(Vector3D(0,-9.8/1,0));
			 s->update(dt);
		 }
	}

	//getters y seters de params adicionales
	int getNivel() { return nivel; }
	void  setNivel(int n) { nivel = n; }
	int getPuntosN(int n) { return puntos[n]; }
	void setPuntosN(int n, int punt) {puntos[n] = punt; }
	void iniPuntos() {
		for (int i = 0; i < 6; i++) {
			puntos[i] = 0;
		}
	}
	void iniPuntosN(int n) {
		puntos[n] = 0;
	}
	bool getCambio() { return cambio; }
	void setCambio(bool a) { cambio = a; }

	//distancia entre dos Solidos cogiendo sus centros
	double getDistancia(Solido &a, Solido &b) {
		Vector3D aux = a.getPos() - b.getPos();
		return aux.length();
	}
	
	//clamp que limita un parametro n entre a y b
	double clampVal(double n, double a, double b) {
		if (n > b) { return b; }
		else if (n < a) { return a; }
		else { return n; }
	}

	//funcion que devuelve el angulo netre dos vectores
	double findAngle(Vector3D v1, Vector3D v2) {
		double auxCos = (v1 * v2) / (v1.length()*v2.length());
		return acos(auxCos);
	}

	//funcion que devuelve la normal mas parecida de un Cubo normalizado  (pos 0,0,0 rotacion 1,0,0,0) segun el parametro nCheck
	//si hay mas de una devuelve un vector particular Vector3D(95172.321, 0, 0)
	Vector3D normalCercana(Vector3D nCheck) {
		
		//se busca el angulo con cada normal
		double check1a = findAngle(nCheck, Vector3D(1, 0, 0));
		double check2a = findAngle(nCheck, Vector3D(0, 1, 0));
		double check3a = findAngle(nCheck, Vector3D(0, 0, 1));
		double check1b = findAngle(nCheck, Vector3D(-1, 0, 0));
		double check2b = findAngle(nCheck, Vector3D(0, -1, 0));
		double check3b = findAngle(nCheck, Vector3D(0, 0, -1));

		//se encuentra el minimo
		double auxMinIni = min(check1a, min(check2a, min(check3a, min(check1b, min(check2b, check3b)))));
		vector<Vector3D> minRet;

		//se vuelve a comprobar para ver si hay mas de una respuesta
		if (auxMinIni == check1a) { 
			minRet.push_back(Vector3D(1,0,0));
		}
		if (auxMinIni == check2a) {
			minRet.push_back(Vector3D(0, 1, 0));
		}
		if (auxMinIni == check3a) {
			minRet.push_back(Vector3D(0, 0, 1));
		}
		if (auxMinIni == check1b) {
			minRet.push_back(Vector3D(-1, 0, 0));
		}
		if (auxMinIni == check2b) {
			minRet.push_back(Vector3D(0, -1, 0));
		}
		if (auxMinIni == check3b) {
			minRet.push_back(Vector3D(0, 0, -1));
		}

		if (minRet.size() == 1) {
			return minRet.at(0);
		}
		else {
			return Vector3D(95172.321, 0, 0);
		}
	}

	//FUNCION PRINCIPAL:
	void checkCollision() {
		//dos fors para comprobar todas las posibles collisiones con todos los posibles objetos
		for (int i = 0; i < solidos.size(); i++) {
			for (int j = i + 1; j < solidos.size(); j++) {
				//se comprueba el primer elemento de la mascara (si es 0 se ignora la collision)
				if (solidos[i]->getColMaskNum(0) > 0 && solidos[j]->getColMaskNum(0) > 0) {
					//comprobando el segundo elemento de la mascara si es 1 y 1 es una collision potencial entre esferas
					if (solidos[i]->getColMaskNum(1) == 1 && solidos[j]->getColMaskNum(1) == 1) {
						//comprobando el tercer parametro de la mascara de colision se ve si la distancia entre ellas es mayor que sus radios de colision
						if (getDistancia(*solidos[i], *solidos[j]) < (solidos[i]->getColMaskNum(2) + solidos[j]->getColMaskNum(2))) { // distancia < r1+r2 de la mascara
							//ecuacion mov
							//resultados
							//normal de choque
							Vector3D n = solidos[j]->getPos() - solidos[i]->getPos();
							n.normalize();
							//dos vecotres del centro de cada esfera al punto de colision
							Vector3D ra = (solidos[i]->getPos() + (n * solidos[i]->getColMaskNum(2))) - solidos[i]->getPos();
							Vector3D rb = (solidos[j]->getPos() - (n * solidos[j]->getColMaskNum(2))) - solidos[j]->getPos();
							//coeficiente elastico
							double mediaElasR = min(solidos[i]->getElasR(),solidos[j]->getElasR());

							//formula que tiene en cuenta todo lo anerior y calculo el impulos de choque usandolo para calcular la velocidad final de cada esfera
							Vector3D J = CollisionResponse(solidos[i], solidos[j], mediaElasR, ra, rb, n);
							solidos[i]->setVel(solidos[i]->getVel() - (J / solidos[i]->getM()));
							solidos[j]->setVel(solidos[j]->getVel() + (J / solidos[j]->getM()));

							//se aplica una velocidad angular segun el vector de la normal del choque
							solidos[i]->setAngVel(solidos[i]->getAngVel() + Quaterniond(mediaElasR / (solidos[i]->getColMaskNum(2) * 10), ra.crossProduct(n)));
							solidos[j]->setAngVel(solidos[j]->getAngVel() + Quaterniond(mediaElasR / (solidos[j]->getColMaskNum(2) * 10), rb.crossProduct(n)));
							
							//se comprueba que la velocidad no se dispara a valores absurdos
							if (solidos[i]->getVel().length() > 25) {
								Vector3D velAuxi = solidos[i]->getVel();
								velAuxi.normalize();
								solidos[i]->setVel(velAuxi * 25);
							}
							if (solidos[j]->getVel().length() > 25) {
								Vector3D velAuxi = solidos[j]->getVel();
								velAuxi.normalize();
								solidos[j]->setVel(velAuxi * 25);
							}

							//comprobando el ultimo elemento de la mascara se ve si la esfera principal choca con una amarrila o azul
							if (solidos[i]->getColMaskNum(3) == 1) {
								if (solidos[j]->getColMaskNum(3) == 2) {
									solidos[j]->setColMask(solidos[j]->getColMaskNum(0), solidos[j]->getColMaskNum(1), solidos[j]->getColMaskNum(2), 0);
									solidos[j]->setCol(Vector3D(0,0,0));
									puntos[nivel] += 10;
								}
								if (solidos[j]->getColMaskNum(3) == 3) {
									solidos[j]->setColMask(solidos[j]->getColMaskNum(0), solidos[j]->getColMaskNum(1), solidos[j]->getColMaskNum(2), 0);
									solidos[j]->setCol(Vector3D(0, 0, 0));
									puntos[nivel] += 50;
									solidos[i]->setVel(solidos[i]->getVel() * 2);
								}
							}
							if (solidos[i]->getColMaskNum(3) == 2) {
								if (solidos[j]->getColMaskNum(3) == 1) {
									solidos[i]->setColMask(solidos[i]->getColMaskNum(0), solidos[i]->getColMaskNum(1), solidos[i]->getColMaskNum(2), 0);
									solidos[i]->setCol(Vector3D(0, 0, 0));
									puntos[nivel] += 10;
								}
							}if (solidos[i]->getColMaskNum(3) == 3) {
								if (solidos[j]->getColMaskNum(3) == 1) {
									solidos[i]->setColMask(solidos[i]->getColMaskNum(0), solidos[i]->getColMaskNum(1), solidos[i]->getColMaskNum(2), 0);
									solidos[i]->setCol(Vector3D(0, 0, 0));
									puntos[nivel] += 50;
									solidos[j]->setVel(solidos[i]->getVel() * 2);
								}
							}
						}
					}
					//caso esfera-cubo (procedimiento parecido al de arriba pero con ciertas diferencias)
					else if (solidos[i]->getColMaskNum(1) == 1 && solidos[j]->getColMaskNum(1) == 2) {
						if (getDistancia(*solidos[i], *solidos[j]) < (solidos[i]->getColMaskNum(2) + (1.5 * pow(solidos[j]->getColMaskNum(2),2)))) { //se asume que el cubo es una esfera para no tener que calcular todo el rato las variables siguientes

							//calculos para ver si se tocan de verdad
							//se normaliza el sistema cubo-esfera, rotando y moviendo la esfera de manera contraria alas rotaciones y transformacioens del cubo
							Quaterniond esferaCuboCentradosQuat = solidos[i]->getPos() - solidos[j]->getPos();
							esferaCuboCentradosQuat = solidos[j]->getRotq().inverseRet() * esferaCuboCentradosQuat * solidos[j]->getRotq(); // rotacion inversa por eso q-1*v*q

							//usando clamps (metodo explicado en pdf) se obtiene el punto mas cercano del cubo respecto a la esfera
							double cubeLimit = ((double)(solidos[j]->getColMaskNum(2))) / 2;
							Quaterniond puntoCuboMasCercano(0,clampVal(esferaCuboCentradosQuat.getI(), -cubeLimit, cubeLimit),
															 clampVal(esferaCuboCentradosQuat.getJ(), -cubeLimit, cubeLimit),
															 clampVal(esferaCuboCentradosQuat.getK(), -cubeLimit, cubeLimit)); // respecto coordenadas normalizadas (ijk -> xyz)

							Quaterniond normalAuxPCercano(puntoCuboMasCercano);

							//todo se desnormaliza
							puntoCuboMasCercano = solidos[j]->getRotq() * puntoCuboMasCercano * solidos[j]->getRotq().inverseRet();

							Vector3D puntocConexion = puntoCuboMasCercano.toVect();
							Vector3D vectConexion = solidos[i]->getPos() - puntocConexion;

							if (vectConexion.length() <= solidos[i]->getColMaskNum(2)) {  //por fin se comprueba si hay colision exacta con el punto obtenido

								//ecuacion mov
								//resultados
								//proceis similar al anterior con una diferencia
								Vector3D ra = puntocConexion - solidos[i]->getPos(); 
								Vector3D rb = puntocConexion - solidos[j]->getPos();
								Vector3D n;

								//se calcula la normal segun la normal de cubo mas cercana, es decir se coge la normal de la cara correspondiente
								Vector3D anguloMin = normalCercana(-normalAuxPCercano.toVect()); //checkear con vectores normalizados
								
								//se hay mas de una normal correspodniente = choque con esquina, se asume que la esquina es un punto de normal esfera-cubo
								if (anguloMin.getX() == 95172.321) {
									n = solidos[j]->getPos() - solidos[i]->getPos();
									//n = Vector3D(-1, 0, 0);
								}
								else {
									Quaterniond auxNquat = solidos[j]->getRotq() * Quaterniond(anguloMin) * solidos[j]->getRotq().inverseRet();
									n = auxNquat.toVect();
									//n = Vector3D(1, 0, 0);
								}

								double mediaElasR = min(solidos[i]->getElasR(), solidos[j]->getElasR());

								Vector3D J = CollisionResponse(solidos[i], solidos[j], mediaElasR, ra, rb, n); //se aplica la media elastica de cada objeto
								solidos[i]->setVel(solidos[i]->getVel() - (J / solidos[i]->getM()));
								solidos[j]->setVel(solidos[j]->getVel() + (J / solidos[j]->getM()));

								solidos[i]->setAngVel(solidos[i]->getAngVel() + Quaterniond(mediaElasR / (solidos[i]->getColMaskNum(2) * 10), ra.crossProduct(n)));
								solidos[j]->setAngVel(solidos[j]->getAngVel() + Quaterniond(mediaElasR / (solidos[j]->getColMaskNum(2) * 10), rb.crossProduct(n)));


								if (solidos[i]->getVel().length() > 25) {
									Vector3D velAuxi = solidos[i]->getVel();
									velAuxi.normalize();
									solidos[i]->setVel(velAuxi * 25);
								}
								if (solidos[j]->getVel().length() > 25) {
									Vector3D velAuxi = solidos[j]->getVel();
									velAuxi.normalize();
									solidos[j]->setVel(velAuxi * 25);
								}

								//puntuacion no parte de collisiones (se pierde si se choca con un cubo)
								if (solidos[i]->getColMaskNum(3) == 1) {
									if (solidos[j]->getColMaskNum(3) == 4) {
										cout << "derrota" << endl;
										puntos[nivel] = 0;
										nivel = nivel - 1;
										cambio = true;
									}
								}
							}
						}
					}
					//colision cubo-esfera igual que arriba
					else if (solidos[i]->getColMaskNum(1) == 2 && solidos[j]->getColMaskNum(1) == 1) {
						if (getDistancia(*solidos[i], *solidos[j]) < (solidos[j]->getColMaskNum(2) + (1.5 * pow(solidos[i]->getColMaskNum(2), 2)))) { // distancia < r1+r2 de la mascara!!

							//calculos anteriores para ver si se tocan de verdad
							Quaterniond esferaCuboCentradosQuat = solidos[j]->getPos() - solidos[i]->getPos();
							esferaCuboCentradosQuat = solidos[i]->getRotq().inverseRet() * esferaCuboCentradosQuat * solidos[i]->getRotq(); // rotacion inversa por eso q-1*v*q

							double cubeLimit = ((double)(solidos[i]->getColMaskNum(2))) / 2;
							Quaterniond puntoCuboMasCercano(0, clampVal(esferaCuboCentradosQuat.getI(), -cubeLimit, cubeLimit),
															  clampVal(esferaCuboCentradosQuat.getJ(), -cubeLimit, cubeLimit),
															  clampVal(esferaCuboCentradosQuat.getK(), -cubeLimit, cubeLimit)); 

							Quaterniond normalAuxPCercano(puntoCuboMasCercano);

							puntoCuboMasCercano = solidos[i]->getRotq() * puntoCuboMasCercano * solidos[i]->getRotq().inverseRet();
							puntoCuboMasCercano = puntoCuboMasCercano + solidos[i]->getPos(); 

							Vector3D puntocConexion = puntoCuboMasCercano.toVect();
							Vector3D vectConexion = solidos[j]->getPos() - puntocConexion;

							if (vectConexion.length() <= solidos[j]->getColMaskNum(2)) {  
																						  //ecuacion mov
																						  //resultados
								Vector3D ra = puntocConexion - solidos[j]->getPos();
								Vector3D rb = puntocConexion - solidos[i]->getPos();
								Vector3D n;

								//normal
								Vector3D anguloMin = normalCercana(normalAuxPCercano.toVect()); //checkear con vectores normalizados

								if (anguloMin.getX() == 95172.321) {
									n = solidos[j]->getPos() - solidos[i]->getPos();
									//n = Vector3D(-1, 0, 0);
								}
								else {
									Quaterniond auxNquat = solidos[i]->getRotq() * Quaterniond(anguloMin) * solidos[i]->getRotq().inverseRet();
									n = auxNquat.toVect();
									//n = Vector3D(1, 0, 0);
								}

								double mediaElasR = min(solidos[i]->getElasR(), solidos[j]->getElasR());

								Vector3D J = CollisionResponse(solidos[i], solidos[j], mediaElasR, ra, rb, n); 
								solidos[i]->setVel(solidos[i]->getVel() - (J / solidos[i]->getM()));
								solidos[j]->setVel(solidos[j]->getVel() + (J / solidos[j]->getM()));
								solidos[i]->setAngVel(solidos[i]->getAngVel() + Quaterniond(mediaElasR / (solidos[i]->getColMaskNum(2) * 10), ra.crossProduct(n)));
								solidos[j]->setAngVel(solidos[j]->getAngVel() + Quaterniond(mediaElasR / (solidos[j]->getColMaskNum(2) * 10), rb.crossProduct(n)));
								
								if (solidos[i]->getVel().length() > 25) {
									Vector3D velAuxi = solidos[i]->getVel();
									velAuxi.normalize();
									solidos[i]->setVel(velAuxi * 25);
								}
								if (solidos[j]->getVel().length() > 25) {
									Vector3D velAuxi = solidos[j]->getVel();
									velAuxi.normalize();
									solidos[j]->setVel(velAuxi * 25);
								}

								//puntuacion no parte de collisiones (se pierde si se choca con un cubo)
								if (solidos[j]->getColMaskNum(3) == 1) {
									if (solidos[i]->getColMaskNum(3) == 4) {
										cout << "derrota" << endl;
										puntos[nivel] = 0;
										nivel = nivel - 1;
										cambio = true;
									}
								}
							}
						}
					}
					//choque cubo-cubo. Se asume que cada cubo es una esfera de radio s/2 y se repite el if de mas arriba
					else if (solidos[i]->getColMaskNum(1) == 2 && solidos[j]->getColMaskNum(1) == 2) {
						if (getDistancia(*solidos[i], *solidos[j]) < (solidos[i]->getColMaskNum(2)/2 + solidos[j]->getColMaskNum(2)/2)) {
							//ecuacion mov
							//resultados
							Vector3D n = solidos[j]->getPos() - solidos[i]->getPos();
							n.normalize();
							Vector3D ra = (solidos[i]->getPos() + (n * solidos[i]->getColMaskNum(2)/2)) - solidos[i]->getPos();
							Vector3D rb = (solidos[j]->getPos() - (n * solidos[j]->getColMaskNum(2)/2)) - solidos[j]->getPos();
							double mediaElasR = min(solidos[i]->getElasR(), solidos[j]->getElasR());

							Vector3D J = CollisionResponse(solidos[i], solidos[j], mediaElasR, ra, rb, n);
							solidos[i]->setVel(solidos[i]->getVel() - (J / solidos[i]->getM()));
							solidos[j]->setVel(solidos[j]->getVel() + (J / solidos[j]->getM()));
							solidos[i]->setAngVel(solidos[i]->getAngVel() + Quaterniond(mediaElasR / (solidos[i]->getColMaskNum(2) * 10), ra.crossProduct(n)));
							solidos[j]->setAngVel(solidos[j]->getAngVel() + Quaterniond(mediaElasR / (solidos[j]->getColMaskNum(2) * 10), rb.crossProduct(n)));

							if (solidos[i]->getVel().length() > 25) {
								Vector3D velAuxi = solidos[i]->getVel();
								velAuxi.normalize();
								solidos[i]->setVel(velAuxi * 25);
							}
							if (solidos[j]->getVel().length() > 25) {
								Vector3D velAuxj = solidos[j]->getVel();
								velAuxj.normalize();
								solidos[j]->setVel(velAuxj * 25);
							}
						}
					}
					else {} // añadir mas colisiones en el futuro
				}
			}
		}
	}
	
	//formula de calculo del impulso ("explicacion" en pdf)
	Vector3D CollisionResponse(Solido *a, Solido *b, double elasR , Vector3D ra , Vector3D rb , Vector3D n) {



		double IaInverse[9];
		for (int i = 0; i < 9; i++) {
			IaInverse[i] = a->getIMTInvNum(i);
		}

		double IbInverse[9];
		for (int i = 0; i < 9; i++) {
			IbInverse[i] = b->getIMTInvNum(i);
		}

		double ma = a->getM();
		double mb = b->getM();

		Vector3D vai = a->getVel();
		Vector3D vbi = b->getVel();

		//inversa ya calculada arriba
		Vector3D normal = n;
		normal.normalize();
		Vector3D angularVelChangea = normal;
		angularVelChangea = angularVelChangea.crossProduct(ra);
		angularVelChangea = angularVelChangea.matPorVector(IaInverse); //matriz3x3 * vector3x1  == vector1x3   da igual se le da la vuelta
		Vector3D vaLinDueToR = angularVelChangea.crossProduct(ra);
		double scalar = 1/ma + (normal*vaLinDueToR);


		Vector3D angularVelChangeb = normal;
		angularVelChangeb = angularVelChangeb.crossProduct(rb);
		angularVelChangeb = angularVelChangeb.matPorVector(IbInverse); //mat33 * vec31 == vec13 ^^
		Vector3D vbLinDueToR = angularVelChangeb.crossProduct(rb);
		scalar += 1/mb + (normal*vbLinDueToR);
		Vector3D JmodPartialVect = (vai - vbi);
		double Jmod = (elasR + 1.0)*JmodPartialVect.length() / scalar;
		Vector3D J = normal * Jmod;

		return J;

	}

	vector<Solido*> getSolidos(){return solidos;}
	void add(Solido *s){solidos.push_back(s);}
	friend std::ostream &operator << (std::ostream &os, const Escena &v);
};
inline std::ostream &operator<<(std::ostream &os, const Escena &e){
	for(Solido *s:e.solidos){
		os << *s <<endl;
	}
    return os;
}
#endif /* ESCENA_H_ */
