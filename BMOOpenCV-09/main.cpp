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

const String caminho = "/Users/brenomedeiros/Documents/ProgramasBMO/Cpp/BMOOpenCV-09/BMOOpenCV-09/";
const String caminhoImagens = caminho + "Images/";
const String caminhoImagensWebcam = caminhoImagens + "webcam/";

const float dimensaoDoQuadradoDaCalibragem = 0.02500f;  //metros
const float dimensaoDoQuadradoAruco = 0.058500f;        //metros
const Size dimensaoTabuleiroXadrex = Size(6,9);


void criarPosicaoDeTabuleiroConhecido(Size tamanhoDoTabuleiro, float comprimentoDeQuadradoDeFronteira, vector<Point3f>& arestas) {
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

//Criado em algum momento entre as aulas 15 e 18, ele foi refeito na Aula 21(Final), e esta neste arquivo.
/*int comecarMonitoramentoDeWebcam(const Mat& matrizDaCamera, const Mat& coeficientesDeDistancia, float dimensaoDoQuadradoAruco) {
    Mat quadro;
    
    vector<int> idsDosMarcadores;
    //Vetor de vetor de Pontos, que são as arestas do marcador que esta sendo detectado.
    vector<vector<Point2f>> arestasDoMarcador, candidatosRejeitados;
    
    aruco::DetectorParameters parametros;
    Ptr<aruco::Dictionary> dicionarioDoMarcador = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    
    VideoCapture vid(0);
    if (!vid.isOpened()) {
        return -1;
    }
    
    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    vector<Vec3d> vetoresRotacionais, vetoresTranslacionais;
    
    //Aqui novamente utilizamos um loop para sair percorrendo a imagem em busca do marcador.
    while (true) {
        if (!vid.read(quadro)) {
            break;
        }

        aruco::detectMarkers(quadro, dicionarioDoMarcador, arestasDoMarcador, idsDosMarcadores);
        aruco::estimatePoseSingleMarkers(arestasDoMarcador, dimensaoDoQuadradoAruco, matrizDaCamera, coeficientesDeDistancia, vetoresRotacionais, vetoresTranslacionais);
        
        for (int i=0; i<idsDosMarcadores.size(); i++) {
            aruco::drawAxis(quadro, matrizDaCamera, coeficientesDeDistancia, vetoresRotacionais, vetoresTranslacionais, 1.0f);
        }
        imshow("Webcam", quadro);
        if (waitKey(30)>=0) {
            break;
        }
    }
    return 1;
}*/

void calibracaoDaCamera(vector<Mat> imagensDeCalibragem, Size tamanhoDoTabuleiro, float comprimentoDeQuadradoDeFronteira, Mat& matrizDaCamera, Mat& coeficientesDeDistancia) {
    vector<vector<Point2f>> pontosEspaciaisDaImagemDoTabuleiro;
    getArestasDoTabuleiro(imagensDeCalibragem, pontosEspaciaisDaImagemDoTabuleiro, false);
    
    //Armazena nossas posicoes de tabuleiro para cada imagem
    vector<vector<Point3f>> PontosDeArestasEspaciaisDoMundo(1); //Tamanho/Size = 1
    criarPosicaoDeTabuleiroConhecido(tamanhoDoTabuleiro, comprimentoDeQuadradoDeFronteira, PontosDeArestasEspaciaisDoMundo[0]);
    
    //Redimensiona essas matrizes para o tamanho das imagens que nos ja temos no elemento da posição 0. Relacionando os pontos 2D com os 3D.
    PontosDeArestasEspaciaisDoMundo.resize(pontosEspaciaisDaImagemDoTabuleiro.size(), PontosDeArestasEspaciaisDoMundo[0]);
    
    vector<Mat> vetorR, vetorT; // vetor de raio, e vetor tangencial
    coeficientesDeDistancia = Mat::zeros(8, 1, CV_64F);
    calibrateCamera(PontosDeArestasEspaciaisDoMundo, pontosEspaciaisDaImagemDoTabuleiro, tamanhoDoTabuleiro, matrizDaCamera, coeficientesDeDistancia, vetorR, vetorT);
}

bool salvarCalibragemDeCamera(string nome, Mat matrizDeCamera, Mat coeficientesDeDistancia) {
    ofstream transmissaoDeSaida(nome);
    if (transmissaoDeSaida) {
        
        uint16_t linhas  = matrizDeCamera.rows;
        uint16_t colunas = matrizDeCamera.cols;
        
        transmissaoDeSaida << linhas << endl;
        transmissaoDeSaida << colunas << endl;
        
        for (int l=0; l<linhas; l++) {
            for (int c=0; c<colunas; c++) {
                double valor = matrizDeCamera.at<double>(l, c);
                transmissaoDeSaida << valor << endl;
            }
        }
        
        linhas = coeficientesDeDistancia.rows;
        colunas = coeficientesDeDistancia.cols;
        
        transmissaoDeSaida << linhas << endl;
        transmissaoDeSaida << colunas << endl;
        
        for (int l=0; l<linhas; l++) {
            for (int c=0; c<colunas; c++) {
                double valor = coeficientesDeDistancia.at<double>(l, c);
                transmissaoDeSaida << valor << endl;
            }
        }
        
        transmissaoDeSaida.close();
        return true;
    }
    
    return false;
}

bool carregarCalibragemDeCamera(string nome, Mat& matrizDeCamera, Mat& coeficientesDeDistancia) {
    ifstream transmissaoDeEntrada(nome);
    if (transmissaoDeEntrada) {
        uint16_t linhas;
        uint16_t colunas;
        
        transmissaoDeEntrada >> linhas;
        transmissaoDeEntrada >> colunas;
        
        matrizDeCamera = Mat(Size(linhas, colunas), CV_64F);    //Inverti a ordem de "(colunas, linhas)" do video, por julgar q estava errado
        
        for (int l=0; l<linhas; l++) {
            for (int c=0; c<colunas; c++) {
                double ler = 0.0f;
                transmissaoDeEntrada >> ler;
                matrizDeCamera.at<double>(l, c) = ler;
                cout << matrizDeCamera.at<double>(l,c) << "\n";
            }
        }
        
        //Coeficientes de Distancia
        transmissaoDeEntrada >> linhas;
        transmissaoDeEntrada >> colunas;
        
        coeficientesDeDistancia = Mat::zeros(linhas, colunas, CV_64F);
        for (int l=0; l<linhas; l++) {
            for (int c=0; c<colunas; c++) {
                double ler = 0.0f;
                transmissaoDeEntrada >> ler;
                coeficientesDeDistancia.at<double>(l,c) = ler;
                cout << coeficientesDeDistancia.at<double>(l,c) << "\n";
            }
        }
        transmissaoDeEntrada.close();
        return true;
    }
    return false;
}

int comecarMonitoramentoDeWebcam(const Mat& matrizDaCamera, const Mat& coeficientesDeDistancia, float dimensaoDoQuadradoAruco) {
    Mat quadro;
    
    vector<int> idsDosMarcadores;
    //Vetor de vetor de Pontos, que são as arestas do marcador que esta sendo detectado.
    vector<vector<Point2f>> arestasDoMarcador, candidatosRejeitados;
    
    aruco::DetectorParameters parametros;
    //Como existem diversos dicionarios, criaremos o ponteiro abaixo para termos certeza de pegar o correto.
    Ptr<aruco::Dictionary> dicionarioDoMarcador = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    
    VideoCapture vid(0);
    if (!vid.isOpened()) {
        return -1;
    }
    
    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    vector<Vec3d> vetoresRotacionais, vetoresTranslacionais;
    
    //Aqui novamente utilizamos um loop para sair percorrendo a imagem em busca do marcador.
    while (true) {
        if (!vid.read(quadro)) {
            break;
        }

        aruco::detectMarkers(quadro, dicionarioDoMarcador, arestasDoMarcador, idsDosMarcadores);
        aruco::estimatePoseSingleMarkers(arestasDoMarcador, dimensaoDoQuadradoAruco, matrizDaCamera, coeficientesDeDistancia, vetoresRotacionais, vetoresTranslacionais);
        
        for (int i=0; i<idsDosMarcadores.size(); i++) {
            aruco::drawAxis(quadro, matrizDaCamera, coeficientesDeDistancia, vetoresRotacionais[i], vetoresTranslacionais[i], 0.1f);
        }
        imshow("Webcam", quadro);
        if (waitKey(30)>=0) {
            break;
        }
    }
    return 1;
}

int processoDeCalibragemDeCamera(Mat& matrizDaCamera, Mat& coeficientesDeDistancia) {
    Mat quadro;
    Mat desenhoProQuadro;
    
    
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
        switch (caractere) {
            case ' ': //Espaco
                //Salvar Imagem.
                if (encontrou) {
                    Mat temp;
                    quadro.copyTo(temp);
                    imagensSalvas.push_back(temp);
                }
                break;
                
            case 13:  //Enter
                //Comecar calibragem, e Salva a Calibragem.
                if (imagensSalvas.size()>15) {
                    calibracaoDaCamera(imagensSalvas, dimensaoTabuleiroXadrex, dimensaoDoQuadradoDaCalibragem, matrizDaCamera, coeficientesDeDistancia);
                    salvarCalibragemDeCamera(caminho+"CalibragemiMac2017.txt", matrizDaCamera, coeficientesDeDistancia);
                }
                break;
                    
            case 'l': //'L'
                //Carrega a Calibragem.
                carregarCalibragemDeCamera(caminho+"CalibragemiMac2017.txt", matrizDaCamera, coeficientesDeDistancia);
                break;
                
            case 27:  //Esc
                //Sair.
                return 0;
                break;
        }
    }
    
    return 0;
}

int main(int argc, const char * argv[]) {
    // insert code here...
    
    Mat matrizDaCamera = Mat::eye(3, 3, CV_64F);
    Mat coeficientesDeDistancia;
    
    processoDeCalibragemDeCamera(matrizDaCamera, coeficientesDeDistancia);
    comecarMonitoramentoDeWebcam(matrizDaCamera, coeficientesDeDistancia, dimensaoDoQuadradoAruco);
    
    return 0;
}
