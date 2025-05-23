# Comment soumettre

## Piste D'autonomie

Les équipes doivent développer leurs solutions (packages ROS) dans un dossier « solutions » du répertoire `~/ros2_ws/src`. Ce dossier peut contenir un ou plusieurs packages ROS.

La structure de répertoire attendue est la suivante :

```
~/ros2_ws/src
.
├── parc_robot
│   ├── CMakeLists.txt
│   └── package.xml
├── parc_robot_bringup
│   ├── .
│   ├── .
│   ├── CMakeLists.txt
│   └── package.xml
├── parc_robot_description
│   ├── .
│   ├── .
│   ├── CMakeLists.txt
│   └── package.xml
├── .
├── .
└── <VOTRE_DOSSIER_SOLUTION>        # Zip ce dossier et soumettre
    ├── <votre_ros_package1>
    │   ├── .
    │   ├── .
    │   ├── setup.py
    │   └── package.xml
    ├── <votre_ros_package2>
    │   ├── .
    │   ├── .
    │   ├── setup.py
    │   └── package.xml
    ├── .
    ├── .
    └── README.md                   # Requis
```
Suivez ces étapes de soumission :

1. Préparez un fichier README.md en suivant ce format et stockez-le dans le dossier de la solution (voir [l'exemple](https://github.com/PARC-Robotics/PARC2025-Engineers-League/blob/main/resources/sample-submission-readme.md)):
     * Section d'introduction : Décrivez brièvement votre approche
     * Dépendances : répertoriez tous les packages installés et utilisés dans votre solution
     * Description de la tâche et commande(s) d'exécution
     * Défis rencontrés

2. Incluez tous les packages (dépendances) utilisés dans votre solution dans le fichier "package.xml" de votre package ([voir guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html){target=_blank})

3. Créez de courtes vidéos de démonstration de votre solution. Vous pouvez le faire en enregistrant votre solution en cours d'exécution dans Gazebo. Assurez-vous que la taille des vidéos est inférieure à 200 Mo.

4. Compressez votre dossier de solution et téléchargez le dossier et les vidéos sur le [formulaire de soumission de solution](https://docs.google.com/forms/d/e/1FAIpQLSdqVseAQhc7G2XDFTknJeDg-ume0ozUMdaQVdp-J3gDF75vjQ/viewform?usp=header){target=_blank}.

## Piste de Conception

Les équipes qui choisissent la voie de la conception doivent suivre les étapes suivantes pour soumettre leur travail :

* Créez un dossier et incluez les fichiers suivants :
    * 3 dessins de chaque pièce de votre outil de fertilisation avec leurs dimensions.
    * Rendus 3D de la conception finale de l'outil. Exportation au format STEP, avec l'extension **.stp**, **.step** ou **.STEP**.
    * Des plans d'assemblage 3D des pièces de l'outil.
    * Une nomenclature des pièces d'assemblage de l'outil.
    * Une vidéo de simulation de mouvement montrant l'outil en fonctionnement. Veuillez vous assurer que la vidéo ne dépasse pas 1 minute et ne pèse pas plus de 50 Mo.
    * Un rapport de conception expliquant votre approche de conception, les considérations prises en compte, les composants de l'outil et son principe de fonctionnement. Le rapport de conception doit inclure le pays de l'équipe et les noms des membres de l'équipe, en commençant par le chef d'équipe.

* Les équipes sont tenues de lire et de signer cet [accord](https://github.com/PARC-Robotics/PARC2025-Engineers-League/blob/main/resources/PARC-Eng-League-Agreement.pdf) qui décrit les conditions relatives aux droits de propriété intellectuelle (PI) et à l'utilisation des travaux soumis. Ce document doit également être inclus dans le dossier de soumission.

* Compressez le dossier contenant les fichiers listés ci-dessus --- au format **.zip**, **.tar** ou **.tar.gz** --- et téléchargez le dossier sur le [formulaire de soumission de solution](https://docs.google.com/forms/d/e/1FAIpQLSdqVseAQhc7G2XDFTknJeDg-ume0ozUMdaQVdp-J3gDF75vjQ/viewform?usp=header){target=_blank}.