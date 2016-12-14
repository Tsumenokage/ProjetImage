#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include <list>

/** @file */ 

using namespace cv;
using namespace std;

#define param 2 /**< Nombre de paramètres attendus lors de l'appel du programme */

float progress = 0.0; /**< Variable utilisé pour la barre de progression */
vector<Mat> images; /**< Vecteur contenant l'ensemble des images du dossier passé en paramètre */
unsigned int image_counter; /**< Entier qui va compter le nombre d'image traité */
RNG rng(12345); /**< Cette variable sera utilisé pour générer aléatoirement des couleurs */
int bordure = 5; /**< Nombre depixel rajouté en bordure de l'image */

/**
 * Structure permettant de définir un point
 * @deprecated
 * */
struct cPoint {
  int x; /** Entier correspondant à l'abscisse du point*/
  int y; /** Entier correspondant à lordonnée du point*/
};

/**
 * Cette fonction renvoie le nombre de pixel non nul d'une image binaire.
 * @param image, une image binaire
 * @return le nombre de pixels non nul de l'image binaire
 **/
int hasValue(Mat image) {
  int count = 0;
  if (image.cols == 0 || image.rows == 0) {
    return 0;
  }
  for(int j=0;j<image.cols;j++) {
    for(int i=0;i<image.rows;i++) {
      if( (int) image.at<uchar>(i,j) != 0)
	count ++;
    }
  }
  return count;
}

/**
 * Récupére tous les pixels contenus dans un carré de coté 2*radius+1 centré sur un pixel de coordonnées désirées sous force de matrice
 * @param image, l'image source
 * @param cols, l'abscisse du pixel sur lequel sera centrée le carré
 * @param rows, l'ordonnée du pixel sur lequel sera centrée le carré
 * @param radius, le rayon de la sous-matrice 
 * @return une matrice de taille au moins 2*radius+1
 * 
 **/
Mat getPixelMatrix(Mat image, int cols, int rows, int radius) {
  int minC, maxC, minR, maxR;
  
  if ( cols-radius < 0) minC = 0; else minC = cols-radius;
  if ( cols+radius > image.cols) maxC = image.cols; else maxC = cols+radius;

  if ( rows-radius < 0) minR = 0; else minR = rows-radius;
  if ( rows+radius > image.rows) maxR = image.rows; else maxR = rows+radius;
 
  Mat submatrix = image.colRange(minC,maxC).rowRange(minR,maxR);    
  return submatrix;
}

/**
 * Par référence, renvoie une matrice binaire de même taille que les 3 autres matrices en paramètres. Pour chaque pixels des 3 matrices src, le pixel associé à la matrice renvoyée est mis à 255 si en appliquant un patch de taille taille_patch: aucun pixel est à 255 sur src1 ET 10*taille_patch pixels sont à 255 sur src2 ET 5*taille_patch pixels sont à 255 sur src3. Si aucun pixels n'a été trouvé et que taille_patch est à 10, la fonction est rappelée avec taille_patch à 5.
 * @param src1 Matrice binaire correspondant à l'herbe
 * @param src2 Matrice binaire correspondant aux objets blancs
 * @param src3 Matrice binaire correspondant au contours du terrain
 * @param dst Référence vers matrice de destination 
 * @param taille_patch taille du patch à appliquer
 * */
void patchAll3(const Mat src1, const Mat src2, const Mat src3, Mat& dst, int taille_patch) {
  dst = Mat::zeros( src1.size(), CV_8UC1);
  int found = 0;
  for(int i=0;i<src1.rows;i++) {
    for(int j=0;j<src1.cols;j++) {
      if( (int) src3.at<uchar>(i,j) == 255) {
	Mat pix1 = getPixelMatrix(src1,j,i,taille_patch);
	Mat pix2 = getPixelMatrix(src2,j,i,taille_patch);
	Mat pix3 = getPixelMatrix(src3,j,i,taille_patch);
	if( (hasValue(pix1) == 0) && (hasValue(pix2) > taille_patch*10) && (hasValue(pix3) > taille_patch*5)) {
	  dst.at<uchar>(i,j) = 255;
	  cout << hasValue(pix1) << " " << hasValue(pix2) << " " << hasValue(pix3) <<endl;
          found = 1;
	} 
      }
    }
  }
  if (!found && taille_patch == 10) patchAll3(src1,src2,src3,dst,5);
}

/**
 * Cette fonction va chercher les contours du terrain
 * @param src Matrice correspondant à l'image dont on veut extraire les contours du terrain
 * @param dst Image permettant d'afficher le contour du terrain
 * 
 * */
void contoursTerrain(const Mat src, Mat &dst) {
  int threshCanny = 100; //Seuil utilisé pour la detection des bords
  vector<vector<Point> > contours; //Matrice de Matrice de points qui contiendras les contours trouvé dans l'image
  Mat detectionTerrainCanny; //Image des bords détectés.
  vector<Vec4i> hierarchy;
  int perimeters = 0; //Plus grand périmètre parmis les contours
  int cntIndex = 0; //Index du plus grands contours
  Mat approxM;

  //Utilisation de Canny pour détecter les bords puis détection des contours
  Canny( src, detectionTerrainCanny, threshCanny, threshCanny*2, 3 );
  findContours( detectionTerrainCanny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  //Utilisation d'un algorithme de Hull pour trouver les enveloppes convexes
  vector<vector<Point> >hull( contours.size());
  for( unsigned int i = 0; i < contours.size(); i++ )
    {  convexHull( Mat(contours[i]), hull[i], false );}
  
  //Dessin de l'ensemble des enveloppes convexes trouvées
  Mat hullP = Mat::zeros( src.size(), CV_8UC3 );
  for( unsigned int i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      //drawContours( hullP, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      drawContours( hullP, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    }
  imshow( "Hull demo", hullP );
  //On cherche la plus grande enveloppe convexe
  for( unsigned int i = 0; i <hull.size();i++)
    {
      int tmpperimeter = arcLength(hull[i],1);
      if(tmpperimeter > perimeters)
        {
          perimeters = tmpperimeter;
          cntIndex = i;
        }
      
    }
  
  //On dessine uniquement l'enveloppe convexe avec le plus grand périmètre
  Mat drawing = Mat::zeros( detectionTerrainCanny.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );     
  drawContours( drawing, hull, cntIndex, color, 2, 8, hierarchy, 0, Point() );

  
  cvtColor(drawing,drawing,CV_BGR2GRAY);
  for(int i=0;i<drawing.rows;i++) {
    for(int j=0;j<drawing.cols;j++) {
      int pixel = drawing.at<uchar>(i,j);
      if(pixel > 0)
	drawing.at<uchar>(i,j) = 255;
    }
  }
  // si ligne du haut avec 255, return pas de contours
  int ok = 0;
  for(int j=50;j<drawing.cols-50;j++) {
    if( (int) drawing.at<uchar>(bordure+1,j) != 255) {
      ok = 1;
      break;
    }
  }
  if (ok == 0)
    dst = Mat::zeros( detectionTerrainCanny.size(), CV_8UC1 );
  else {
    drawing.copyTo(dst);
  }
}

/**
 * Cette fonction va chercher l'ensemble de l'herbe dans une image passé en paramètre
 * @param im Image dont on veut extraire l'herbe
 * @param dst Image binaire dont les zones blanches correspondront à l'herbe trouvé dans im
 * 
 * */
void detectionGrass(const Mat im, Mat &dst) {
  Mat imgTh;
  Mat imHSV;
  //Définition des valeurs min et max de H,S et V pour détecter l'herbe
  int iLowH = 22;
  int iHighH = 60;

  int iLowS = 50; 
  int iHighS = 215;

  int iLowV = 40;
  int iHighV = 175;
  //Converstion de l'image du format BGR a HSV
  cvtColor(im,imHSV,CV_BGR2HSV);
  //Algorithme de recherche d'une plage de couleur dans une image
  inRange(imHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTh);
  imgTh.copyTo(dst);

}

/**
 * Cette fonction va charger l'ensemble des images dans le dossier passé en paramètres
 * @param folder String correspondant au dossier ou seront cherché les images
 * */
void init(String folder) {
  cout << "Initialization" << endl;
  
  
  cv::String path(folder + "/*.png"); //select only jpg
  vector<cv::String> fn;
  glob(path,fn,true); // recurse
  for (size_t k=0; k<fn.size(); ++k)
    {
      cv::Mat im = cv::imread(fn[k]);
      if (im.empty()) continue; //only proceed if sucsessful
      // you probably want to do some preprocessing
      images.push_back(im);
    }
}

/**
 * Cette fonction va vérifier la présence d'herbe sur les bords de l'image. Cette fonction est utilisée 
 * pour vérifier si le robot ne regarde pas a ces pieds et donc ne voit pas les bords du terrain
 * @param grass Une image binaire représentant l'endroit ou se trouve l'herbe
 * @return Un booléen qui vaudra vrai si il y'a trop d'herbe en bordure de l'image (cela signie que le robot regarde
 * à ces pieds) où false dans le cas contraire 
 * */
bool tooMuchGrass(const Mat grass) {
  //On va compter l'ensemble des pixels blancs présents en bordure de l'image
  int green_pixel = 0;
  for (int i=0;i<grass.cols;i++) {
    cout << "first "<<i<<endl;
    if ( (int) grass.at<uchar>(0+bordure,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(1+bordure,i) == 255)
      green_pixel++;

    if ( (int) grass.at<uchar>(grass.rows-1-bordure,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(grass.rows-2-bordure,i) == 255)
      green_pixel++;
  }
  for (int i=0;i<grass.rows;i++) {
    cout << "sec "<< i+bordure <<" "<<grass.rows<<endl;
    if ( (int) grass.at<uchar>(i,0+bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,1+bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-1-bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-2-bordure) == 255)
      green_pixel++;
  }
  int max_pixel = 4 * (grass.cols-(bordure*2) + grass.rows-(bordure*2));
  
  //cout << "green "<< ((float)green_pixel)/max_pixel << endl;
  
  imshow("grass",grass);
 
  if (green_pixel > 0.8 * max_pixel)
    return true;
  return false;
}

/**
 * Cette fonction va permettre d'obtenir l'image suivante dans la liste d'image a traiter
 * @param M Matrice passé en référence qui contiendra l'image suivante 
 * @return Un entier valant 0 si on arrive à la fin de la liste d'image et 1 dans le cas inverse 
 * */
int getNextMatrix(Mat& M) {

  if (image_counter < images.size()) {
    M = images[image_counter];
    //cout << image_counter << endl;
    image_counter++;
    return 1;
  } else {
    return 0;
  }

}

/**
 * Cette fonction va prendre une image binaire passé en paramètre  et va chercher les zones blanches qui correspondent à la base des
 * poteaux détectés. Elle va ensuite entourer ces zones avec des cercles rouges sur l'image d'origine
 * @param src Image binaire représantant les zones ou sont detectés le ou les poteaux des buts
 * @param originale Image originale ou seront dessiné les cercles

 * 
 * */
void contourBlob(Mat src, Mat originale) {
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
	
  //On cherche les contours de chaque zone blanches correspondant à un poteaux
  findContours( src, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  vector<Point> centers;
  vector<float> radiuss;
  //Si on trouve pas trops de zone correspondant à des poteaux 
  if(contours.size() <= 300)
    {
      //On dessine un cercle autour de chaque zone
      for (unsigned int i = 0; i < contours.size(); i++)
        {
          Point2f center;
          float radius;
          minEnclosingCircle(contours[i],center,radius);
          centers.push_back(center);
          radiuss.push_back(radius);
          circle(originale, center, 40, Scalar(0,0,255));
        }
		
		
    }
	
}

/**
 * Cette fonction va gérer l'affichage de la barre de progression
 * */
void progressBar() {
  int barWidth = 70;
	
  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; ++i) {
    if (i < pos) std::cout << "=";
    else if (i == pos) std::cout << ">";
    else std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0) << " %\r";
  std::cout.flush();
	

  std::cout << std::endl;
}

/**
 * Cette fonction va réaliser tout le processus de détections des buts
 * @param benchmark Option permettant de spécifier si l'on veut le type d'affichage voulue
 * 
 * */
void process(String benchmark) {
  clock_t begin = clock();
  bool bench;
    
  if(benchmark == "b")
    {
      bench = true;
    }
  else if(benchmark == "s")
    {
      bench = false;
    }
  else
    {
      cerr << "Wrong option" << endl;
      exit(EXIT_FAILURE);
    }
  
  
  while(1)
    {
	  
      Mat image;
      int found = getNextMatrix(image);
      if (found == 0)
	break;

      /************ Ajout d'une bordure de pixels noirs à l'image *****************************/
      copyMakeBorder(image,image,bordure,bordure,bordure,bordure,BORDER_CONSTANT,Scalar(0,0,0));

      /************ Détection de composante couleur de l'image *******************/
      
      
      /** Détection des poteaux des buts (zones blanches de l'image) **/
      Mat hsv,HS;
      cvtColor(image,hsv,CV_BGR2HSV);
      inRange(hsv, Scalar(20, 175, 0), Scalar(255, 255, 255), HS); //Threshold the image
      Mat elementB = getStructuringElement(MORPH_ELLIPSE, Size(4,4), Point(0, 0) );
      erode(HS,HS,elementB);
      
      /** Détection de l'herbe (zone verte de l'image **/
      Mat grass;
      detectionGrass(image,grass);
      medianBlur(grass,grass,7);
      
      //imshow("grass", grass);
      /** Opération morphologique sur l'herbe afin d'obtenir une zone la plus lisse possible **/
      Mat element = getStructuringElement(MORPH_ELLIPSE, Size(10,10), Point(0, 0) );
      erode(grass,grass,element);
      erode(grass,grass,element);
      dilate(grass,grass,element);
      dilate(grass,grass,element);

      /*** Si trops d'herbe autour, le robot regarde à ses pieds et donc peu de chance d'y voir un poteau ***/
      if ( !tooMuchGrass(grass)) {
		  
        /********* Détection des contours du terrain ************/
        Mat countour;
        contoursTerrain(grass,countour);
      
        /******** On merge les trois composantes : Herbes, Poteaux (zone blanches) et contour du terraind ans une même image**/
        Mat input[3];
        input[0] = HS;
        input[1] = grass;
        input[2] = countour;
        Mat output;
        merge(input,3,output);
        if(!bench)
          imshow("output",output);
      
        /************On cherche les poteaux précisément grâce à la fonction patchAll3**************/
        Mat last;
        patchAll3(grass, HS, countour, last, 10);
      
        /*****On effectue une dilation sur l'image retournée par patchAll3 *******/
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5,5), Point(0, 0) );
        dilate(last,last,element);
        if(!bench)
          imshow("last",last);
        /**** On entoure les zones correspondant à la zone des poteaux trouvées *****/
        contourBlob(last,image);
      
        progress = (float)image_counter/(float)images.size();
        progressBar();
      }
      
      progress = (float)image_counter/(float)images.size();
      progressBar();
      if(!bench)
        {
          imshow("origin",image);
          waitKey();
        }
    }
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
  if(bench)
    {
      cout << "Numbers of pictures : " << images.size() << endl;
      cout << "Computing time : " << elapsed_secs << "s" <<endl;
      cout << "Time per images : " << elapsed_secs/images.size() << "s" << endl;
      cout << "Frames per seconds : " << 1.0/(elapsed_secs/images.size()) << endl;
    }
  exit(EXIT_SUCCESS);
}

/**
 * Fonction qui sera appelé lorsque le nombre de paramètres utilisé pour appelé le programme est incorrecte
 * @param s Le nom du programme
 */
void usage (const char *s) {
  cerr << "Usage " << s << "FileFolder" << "option"<< endl;
  cerr << "option : s (see all pipeline pictures) or b (No images display, use for benchmark)" << endl;
  exit(EXIT_FAILURE);
}

/**
 * Fonction principale du programme 
 */
int main(int argc, char* argv[]) {
  if (argc != (param+1))
    usage(argv[0]);
  init(argv[1]);
  process(argv[2]);
  return EXIT_SUCCESS;
}



/**
 * Colorie l'espace connexe associé au point de coordonnées (x,y) à la couleur color_pixel, en nuance de gris uniquement
 * @param grass Matrice binaire source
 * @param current Matrice de nuance de gris destination
 * @param color_pixel niveau de gris (0-255)
 * @param x abscisse du point désiré
 * @param y ordonnée du point désiré
 * @return Le nombre de pixels de cette espace connexe
 * @deprecated
 * */
int zoneColor(const Mat grass, Mat& current, int color_pixel, int x, int y) {
  list<int> abs;
  list<int> ord;
  abs.push_back(x);
  ord.push_back(y);
  int size = 0;
  while (abs.size() != 0) {
    int i = abs.front();
    int j = ord.front();

    if ( (int) grass.at<uchar>(i,j) == 255 && current.at<uchar>(i,j) == 0) {
      current.at<uchar>(i,j) = color_pixel;
      size++;
      
      //haut
      if( i!=0 && (int) grass.at<uchar>(i-1,j) == 255 && current.at<uchar>(i-1,j) == 0) {
	abs.push_back(i-1);
	ord.push_back(j);
      }

      //bas
      if( i!=(grass.rows-1) && (int) grass.at<uchar>(i+1,j) == 255 && current.at<uchar>(i+1,j) == 0) {
	abs.push_back(i+1);
	ord.push_back(j);
      }

      //gauche
      if( j!=0 && (int) grass.at<uchar>(i,j-1) == 255 && current.at<uchar>(i,j-1) == 0) {
	abs.push_back(i);
	ord.push_back(j-1);
      }

      //droite
      if( j!=(grass.cols-1) && (int) grass.at<uchar>(i,j+1) == 255 && current.at<uchar>(i,j+1) == 0) {
	abs.push_back(i);
	ord.push_back(j+1);
      }

    }
    abs.pop_front();
    ord.pop_front();
  }
  return size;
}

/**
 * Renvoi de quel côté se trouve le point r par rapport au vecteur pq
 * @param p Point
 * @param q Point 
 * @param r Point
 * @return 0 si les trois points sont alignés, 1 si r est à gauche du vecteur pq, 0 sinon
 * @deprecated
 * */
int orientation(cPoint p, cPoint q, cPoint r) {
  int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (val == 0)
    return 0;
  return (val>0) ? 1:2;
}

/**
 * Implémentation de la marche de Jarvis permettant de trouver et d'afficher l'enveloppe connexe d'un ensemble de points. Cette fonction n'est pas utilisée en effet sa complexité dépend linéairement du nombre de points à traiter, environ 300 000 points, rend la durée de traitement trop grande.
 * @param points tableau des points 
 * @param n taille du tableau
 * @param grass image binaire source
 * @deprecated
 * */
void jarvisSlave(cPoint points[], int n, const Mat grass) {
  cout << "begin slave" << endl;
  if(n<3) return;
  int next[n];

  // init
  for(int i=0;i<n;i++) {
    next[i] = -1;
  }
  // Find the leftmost point
  int l = 0;
  for (int i = 1; i < n; i++)
    if (points[i].x < points[l].x)
      l = i;

  // Start from leftmost point, keep moving counterclockwise
  // until reach the start point again

  cout << "begin boucle" << endl;
  int p = l, q;
  do {
    // Search for a point 'q' such that orientation(p, i, q) is
    // counterclockwise for all points 'i'
    q = (p + 1) % n;
    for (int i = 0; i < n; i++)
      if (orientation(points[p], points[i], points[q]) == 2)
	q = i;
    next[p] = q; // Add q to result as a next point of p
    p = q; // Set p as q for next iteration
  }
  while (p != l);
  cout << "end boucle" << endl;
  
  //display
  Mat jarvis_mat = Mat::zeros( grass.size(), CV_8UC1);
  for(int i =0; i<n;i++) {
    if(next[i] + -1) {
      jarvis_mat.at<uchar>(points[i].x,points[i].y) = 255;
    }
  }
  imshow("jarvis",jarvis_mat);
}

/**
 * Interface entre la représentation de données d'opencv et de la marche de jarvis implémentée, voir jarvisSlave.
 * @param grass Matrice binaire contenant les points à envelopper
 * @param size Nombre de pixels à 255.
 * @deprecated
 * */	
void jarvis(const Mat grass, int size) {
  cPoint points[size];
  int cur = 0;
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      if ( (int) grass.at<uchar>(i,j) == 255) {
	points[cur].x = i;
	points[cur].x = j;
	cur++;
      }
    }
  }
  jarvisSlave(points,size,grass);
}

/**
 * Par référence, renvoie le plus grand élément connexe d'une image binaire entrée en paramètre. Ne foncionne uniquement si moins de 255 éléments connexes.
 * @param grass Matrie source
 * @param dst Matrice destination
 * @deprecated
 * */
void grassProcessing(const Mat grass, Mat & dst)  {
  Mat current;
  current = Mat::zeros( grass.size(), CV_8UC1);
  int color_pixel = 1;
  int biggest_size = 0;
  int biggest_color = 1;
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      cout << i << " hey " << j << endl;
      if ( (int) grass.at<uchar>(i,j) == 255 && (int) current.at<uchar>(i,j) == 0 ) {
	int size_color = zoneColor(grass,current, color_pixel,i,j);
	if (size_color > biggest_size) {
	  biggest_size = size_color;
	  biggest_color = color_pixel;
	}
	color_pixel++;
      }
    }
  }
  cout << "color max " << color_pixel <<endl;

  cout << "size" << grass.cols*grass.rows<< " max" << biggest_size << endl;
  imshow("current_green",current);

  Mat biggest =  Mat::zeros( grass.size(), CV_8UC1);
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      if ( (int) current.at<uchar>(i,j) == biggest_color) {
	biggest.at<uchar>(i,j) = 255;
      }
    }
  }

  biggest.copyTo(dst);
  imshow("biggest_green",biggest);
}
	
