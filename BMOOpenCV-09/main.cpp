//
//  main.cpp
//  BMOOpenCV-09
//
//  Created by Breno Medeiros on 05/04/20.
//  Copyright © 2020 ProgramasBMO. All rights reserved.
//

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const String caminho = "/Users/brenomedeiros/Documents/ProgramasBMO/Cpp/BMOOpenCV-08/BMOOpenCV-08/";
const String caminhoImagens = caminho + "Images/";
const String caminhoImagensWebcam = caminhoImagens + "webcam/";

const float dimensaoDoQuadradoDaCalibragem = 0.02500f;  //metros
const float dimensaoDoQuadradoAruco = 0.058500f;        //metros
const Size dimensaoTabuleiroXadrex = Size(6,9);


void criarTabuleiroConhecido(Size tamanhoDoTabuleiro, float comprimentoDeQuadradoDeFronteira, vector<Point3f>& arestas) {
    for (int i=0; i<tamanhoDoTabuleiro.height; i++) {
        for (int j=0; j<tamanhoDoTabuleiro.width; j++) {
            arestas.push_back(Point3f(j*comprimentoDeQuadradoDeFronteira, i*comprimentoDeQuadradoDeFronteira, 0.0f));
        }
    }
}

void getArestasDoTabuleiro(vector<Mat> imagens, vector<vector<Point2f>>& todasAsArestasEncontradas, bool exibirResultados=false) {
    for (vector<Mat>::iterator iter=imagens.begin(); iter!=imagens.end(); iter++) {
        vector<Point2f> bufDoPonto;
        bool encontrou = findChessboardCorners(*iter, dimensaoTabuleiroXadrex, bufDoPonto, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        if (encontrou) {
            todasAsArestasEncontradas.push_back(bufDoPonto);
        }
        
        if (exibirResultados) {
            drawChessboardCorners(*iter, dimensaoTabuleiroXadrex, bufDoPonto, encontrou);
            imshow("Buscando por arestas", *iter);
            waitKey(0);
        }
    }
}

int main(int argc, const char * argv[]) {
    // insert code here...
    
    Mat quadro;
    Mat desenhoProQuadro;
    
    Mat matrizDaCamera = Mat::eye(3, 3, CV_64F);
    
    Mat coeficienteDeDistancia;
    
    vector<Mat> imagensSalvas;
    
    //Vetor que detecta as arestas dos marcadores que estamos preparando.
    vector<vector<Point2f>> arestasDosMarcadores, candidatosRejeitados;
    
    VideoCapture vid(0);
    if (!vid.isOpened()) {
        return 0;
    }
    int quadrosPorSegundo=20;
    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    
    while (true) {
        if (!vid.read(quadro)) {
            break;
        }
        vector<Vec2f> pontosEncontrados;
        bool encontrou=false;
        
        encontrou = findChessboardCorners(quadro, dimensaoTabuleiroXadrex, pontosEncontrados, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        quadro.copyTo(desenhoProQuadro);
        
        drawChessboardCorners(desenhoProQuadro, dimensaoTabuleiroXadrex, pontosEncontrados, encontrou);
        if (encontrou) {
            imshow("Webcam", desenhoProQuadro);
        } else {
            imshow("Webcam", quadro);
        }
        char caractere = waitKey(1000/quadrosPorSegundo);
    }
    
    return 0;
}
