//entrainement modele
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <iostream>

int main() {
    // Définir les paramètres du modèle LBPH
    int radius = 1;
    int neighbors = 8;
    int grid_x = 8;
    int grid_y = 8;
    double threshold = 100.0;

    // Créer un vecteur pour stocker les images de visages et les étiquettes correspondantes
    std::vector<cv::Mat> images;
    std::vector<int> labels;

    // Charger la vidéo
    cv::VideoCapture video("Video.mp4");
    if (!video.isOpened()) {
        std::cerr << "ERREUR : Impossible de charger la vidéo" << std::endl;
        return 1;
    }

    // Variables pour le contrôle de la durée de la vidéo
    double fps = video.get(cv::CAP_PROP_FPS);
    int duration = 14;  // Durée en secondes
    int numFrames = duration * fps;

    // Parcourir la vidéo et extraire les images de visages à intervalles réguliers
    for (int i = 0; i < numFrames; ++i) {
        cv::Mat frame;
        video >> frame;

        // Convertir l'image en niveaux de gris pour la détection des visages
        cv::Mat grayFrame;
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

        // Détection des visages dans l'image
        cv::CascadeClassifier faceCascade;
        if (!faceCascade.load("haarcascade_frontalface_default.xml")) {
            std::cerr << "ERREUR : Impossible de charger le classificateur Haar pour les visages" << std::endl;
            return 1;
        }

        std::vector<cv::Rect> faces;
        faceCascade.detectMultiScale(grayFrame, faces, 1.1, 5, 0, cv::Size(80, 80));

        // Si un visage est détecté, l'ajouter aux images de visages
        if (!faces.empty()) {
            // Récupérer le visage détecté
            cv::Mat faceROI = grayFrame(faces[0]);

            // Redimensionner le visage à une taille fixe si nécessaire
            cv::Mat resizedFace;
            cv::resize(faceROI, resizedFace, cv::Size(100, 100));

            // Ajouter l'image du visage avec son étiquette dans les vecteurs
            images.push_back(resizedFace);
            labels.push_back(i + 1);  // Étiquette du visage, ici on utilise simplement le numéro de l'image
        }
    }

    // Créer le modèle de reconnaissance LBPH
    cv::Ptr<cv::face::LBPHFaceRecognizer> recognizer = cv::face::LBPHFaceRecognizer::create(radius, neighbors, grid_x, grid_y, threshold);

    // Entraîner le modèle avec les images et les étiquettes
    recognizer->train(images, labels);

    // Enregistrer le modèle dans un fichier
    recognizer->save("modele.yml");

    std::cout << "Entraînement terminé. Le modèle a été enregistré." << std::endl;

    return 0;
}

