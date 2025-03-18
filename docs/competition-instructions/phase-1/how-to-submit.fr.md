# Comment soumettre

Les équipes devraient développer leurs solutions (packages ROS) dans un dossier 'solutions' à l'intérieur du répertoire `~/catkin_ws/src`. Vous pouvez avoir un ou plusieurs packages ROS dans ce dossier pour toutes vos tâches.

Voir la figure ci-dessous de la structure du répertoire attendu:

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
├── parc_robot_interfaces
│   ├── msg
│   │   └── CropYield.msg
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

1. Préparez un fichier README.md en suivant ce format et stockez-le dans le dossier de la solution (voir [l'exemple](https://github.com/PARC-Robotics/PARC2024-Engineers-League/blob/main/resources/sample-submission-readme.md)):
     * Section d'introduction : Décrivez brièvement votre approche
     * Dépendances : répertoriez tous les packages installés et utilisés dans votre solution
     * Description de la tâche 1 - 2 et commande(s) d'exécution
     * Défis rencontrés

2. Incluez tous les packages (dépendances) utilisés dans votre solution dans le fichier "package.xml" de votre package ([voir guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html){target=_blank})

3. Créez de courtes démos vidéo simples de vos solutions pour les tâches 1 et 2. Cela peut être fait en prenant un enregistrement d'écran de votre solution en cours d'exécution dans Gazebo. Veuillez vous assurer que les vidéos ont une taille inférieure à 200 Mo.

<!-- 4. Compressez votre dossier de solution et téléchargez le dossier et les vidéos sur le [formulaire de soumission de solution](https://forms.gle/GwE7Tzm9FpYzUVQX9). -->
4. Compressez votre dossier de solution et téléchargez le dossier et les vidéos sur le formulaire de soumission de solution (À FOURNIR)