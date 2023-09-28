SINGULAR FACE TRACKING ROBOT

Le but était de créer un robot qui cherche puis suit le seul visge qu'il connaît. Et ce peu importe s'il y a d'autres personne autour.

Le projet est constitué de deux codes, un premier servant à entraîner un modèle sur la base d'une vidéo et le deuxième se servant du fichier .yml créé pour reconnaître grâce à la caméra le visage recherché puis le suivre à l'aide de deux moteurs.

Ces deux programmes sont codés en C++, ils necessitent l'installation d'opencv 4 ainsi que l'installation du logiciel arduino. En effet, il faudra d'abord sélectionner sur celui-ci le port correspondant et upload sur la carte Arduino Uno le programme "Servo_Parse_int.ino". Une fois ceci fait, on s'attaquera à la création du modèle.

Dans le dossier "training", il faudra placer une vidéo de la personne à reconnaitre d'environ 15 sec et la nommer "video.mp4". Ligne 27 du fichier "training.cpp", il faudra y noter la durée précise. Dans le terminal du dossier, on lance le makefile puis le programme. Un fichier nommé "modele.yml" vient d'être créé. Ce fichier il faut le placer dans le dossier "prog". Dans le fichier "prog.cpp", ligne 138 il faut remplacer le chiffre pour bien être sur le bon port USB pour la caméra et pour information la webcam correspond au 0. Ensuite, ligne 86 on écrira le port trouvé precedemment sur le logiciel arduino. Une fois ceci fait, il ne reste plus qu'a lancer le make et le programme pour que le robot se mette en marche.

Pierre&Louis
