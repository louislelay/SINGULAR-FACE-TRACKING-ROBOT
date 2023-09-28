//suivi visagelouis


#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

using namespace cv;
using namespace std;


int stop = 1;
int Sx = 90;
int Sy = 25;

void sighandle(int sig) // can be called asynchronously
{
	stop = 0;
}

void serial_init(int fd) {
	struct termios serial;
	// Configure serial port settings
	memset(&serial, 0, sizeof(serial));
	cfsetspeed(&serial, B9600); // Baud rate

	serial.c_cflag &= ~PARENB;                 // No parity
	serial.c_cflag &= ~CSTOPB;                 // 1 stop bit
	serial.c_cflag &= ~CSIZE;                  // Mask data size
	serial.c_cflag |= CS8;                     // 8 data bits
	serial.c_cflag &= ~CRTSCTS;                // no flow control
	serial.c_cflag |= CREAD | CLOCAL;          // turn on READ & ignore ctrl lines
	serial.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	serial.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	serial.c_oflag &= ~OPOST;                          // make raw

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &serial);
	return;
}

float mapf(float x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ssize_t serial_read_until(int fd, char *buf, char until) {
	char b[1];
	int i = 0;
	do {
		int n = read(fd, b, 1); // read a char at a time
		if (n == -1)
			return -1; // couldn't read
		if (n == 0) {
			usleep(10 * 1000); // wait 10 msec try again
		continue;
    		}
    		buf[i] = b[0];
    		i++;
  	} while (b[0] != until);
  	buf[i] = 0; // null terminate the string
  	return i + 1;
}

void *moteurs(void *vargp) {
	//pthread_detach(pthread_self());
	printf("Test\n");
	int arduino;
	signal(SIGINT, sighandle);
	// Open the serial port (replace "/dev/ttyUSBX" with your Arduino's
	// port)
	//arduino = open("/dev/cu.usbmodem1101", O_RDWR | O_NOCTTY);
	arduino = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	if (arduino == -1) {
		perror("Error opening serial port");
		return vargp;
	}
	serial_init(arduino);
	// Write data to Arduino
	// char message[] = "AAA\r\n100 1\r\n";
	// write(arduino, message, strlen(message));
	// Read data from Arduino
	char buffer[255];
	while (stop) {
		char message[10];
		sprintf(message, "%d %d\n", Sy, Sx);
		write(arduino, message, strlen(message));
		usleep(200 * 1000); // sleep for 100 msec

		// Display received data and send every 10th received data back to
		// ssize_t numBytes = serial_read_until(arduino, buffer, '\n');
		ssize_t numBytes = read(arduino, buffer, sizeof(buffer) - 1);
		if (numBytes > 0) {
			buffer[numBytes] = '\0';
			printf("Received data: %s\n", buffer);
		}
	}
	// Close the serial port
	close(arduino);
	return vargp;
}




int main() {


	pthread_t thread_id;
	printf("Before Thread\n");
	pthread_create(&thread_id, NULL, moteurs, NULL);
	printf("After Thread\n");
    
	int x = 0;
	int y = 0;

	int dx = 0;
	int dy = 0;

	// Charger le modèle LBPH depuis le fichier YAML
	cv::Ptr<cv::face::LBPHFaceRecognizer> recognizer = cv::face::LBPHFaceRecognizer::create();
	recognizer->read("modele.yml");

	// Ouvrir la webcam
	cv::VideoCapture webcam(0);
	if (!webcam.isOpened()) {
		std::cerr << "ERREUR : Impossible d'ouvrir la webcam" << std::endl;
		return 1;
	}

	// Créer le classificateur de visages Haar
	cv::CascadeClassifier faceCascade;
	if (!faceCascade.load("haarcascade_frontalface_default.xml")) {
		std::cerr << "ERREUR : Impossible de charger le classificateur Haar pour les visages" << std::endl;
		return 1;
	}

	// Dimensions de la fenêtre d'affichage
	int windowWidth = webcam.get(cv::CAP_PROP_FRAME_WIDTH);
	int windowHeight = webcam.get(cv::CAP_PROP_FRAME_HEIGHT);

	// Origine du repère
	int originX = windowWidth / 2;
	int originY = windowHeight / 2;

	// Boucle principale pour la détection et la reconnaissance des visages
	while (true) {
		// Lire une image depuis la webcam
		cv::Mat frame;
		webcam >> frame;

		// Convertir l'image en niveaux de gris pour la détection des visages
		cv::Mat grayFrame;
		cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

		// Détection des visages dans l'image
		std::vector<cv::Rect> faces;
		faceCascade.detectMultiScale(grayFrame, faces, 1.1, 5, 0, cv::Size(80, 80));

		// Variables pour le plus grand rectangle détecté
		cv::Rect largestRect;
		int largestRectSize = 0;

		// Parcourir les visages détectés et les reconnaître
		for (const auto& faceRect : faces) {


			// Récupérer le visage détecté
			cv::Mat faceROI = grayFrame(faceRect);

			// Redimensionner le visage à une taille fixe pour la reconnaissance
			cv::Mat resizedFace;
			cv::resize(faceROI, resizedFace, cv::Size(100, 100));

			// Reconnaître le visage à l'aide du modèle LBPH
			int predictedLabel = recognizer->predict(resizedFace);

			//impose une note minimum
			if (predictedLabel>10){

				// Vérifier si le rectangle actuel est plus grand que le plus grand rectangle précédent
				if (faceRect.width * faceRect.height > largestRectSize) {
					largestRect = faceRect;
					largestRectSize = faceRect.width * faceRect.height;
				}
				// Dessiner un rectangle autour du visage détecté
				cv::rectangle(frame, faceRect, cv::Scalar(0, 255, 0), 2);

				// Afficher le label prédit à côté du visage
				cv::Point labelPosition(faceRect.x, faceRect.y - 10);
				cv::putText(frame, std::to_string(predictedLabel), labelPosition, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            	
				// Calculer les coordonnées du centre du plus grand rectangle détecté
				x = largestRect.x + largestRect.width / 2 - originX;
				y = largestRect.y + largestRect.height / 2 - originY;

				// Dessiner un cercle au milieu du rectangle
				cv::circle(frame, cv::Point(x + originX, y + originY), 5, cv::Scalar(255, 0, 0), -1);
		
				dx = mapf((float)x, -700, 700, -10, 10);
				dy = mapf((float)y, -700, 700, -10, 10);
			
				if(x==-640 && y==-480){}
				else{
					Sy-=dy;
					Sx-=dx;
				}
				if (Sx > 179){Sx = 179;};
				if (Sx < 1){Sx = 1;};
				if (Sy > 179){Sy = 179;};
				if (Sy < 1){Sy = 1;};
			}
        
		}
	
	

	
	
		// Afficher les coordonnées du milieu du rectangle
		std::string coordinates = "Coordonnees : (" + std::to_string(x) + ", " + std::to_string(y) + ")";
		cv::putText(frame, coordinates, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);

		// Afficher l'image avec les rectangles de visages détectés
		cv::imshow("Détection des visages", frame);

		// Attendre la touche 'q' pour quitter
		if (cv::waitKey(1) == 'q') {
			break;
		}
	}

	// Fermer la fenêtre et libérer les ressources
	cv::destroyAllWindows();
	webcam.release();

	return 0;
}
