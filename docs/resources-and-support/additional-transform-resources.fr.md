# Ressources supplémentaires sur les transformations

## Introduction

Pour résoudre avec succès le défi de la détection des mauvaises herbes (Tâche 2), vous devrez comprendre comment utiliser les transformations. Cette page fournit des ressources supplémentaires sur les transformations. Bien qu'il puisse y avoir d'autres moyens de résoudre le défi, nous vous suggérons de considérer ces séries de transformations pour résoudre le défi (voir la figure ci-dessous pour l'illustration correspondante) :

![Transforms](assets/transforms.png)

### Transformer 1 (Cadre de l'image au cadre de la caméra)
Tout d'abord, vous devrez transformer les emplacements des mauvaises herbes du point de vue de la caméra au cadre de référence de la caméra. Cela se fait en projetant les points 2D (u,v -- pixels) dans le plan de l'image dans les points cartésiens 3D (x,y,z -- mètres) en utilisant une transformation de perspective.
Pour effectuer la transformation du plan image en coordonnées cartésiennes dans le cadre de la caméra, vous aurez besoin de quelques propriétés de caméra que vous pouvez trouver [ici](https://github.com/PARC-Robotics/PARC-Engineers-League/blob/master /parc_robot/urdf/side_cameras.xacro).

Les ressources pertinentes pour Transform 1 peuvent être trouvées ici :

- [Calibrage de la caméra et reconstruction 3D](https://docs.opencv.org/2.4.13/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
- [CS231A Course Notes 1: Camera Models](https://web.stanford.edu/class/cs231a/course_notes/01-camera-models.pdf)


### Transformer 2 (image de la caméra en image du robot)
Ensuite, vous devrez transformer les emplacements des mauvaises herbes du cadre de référence de la caméra au cadre de référence du robot. Cela nécessiterait d'utiliser le module de transformation ROS tf (voir liens utiles ci-dessous).

### Transformer 3 (image du robot en image du monde)
Enfin, vous devrez transformer les emplacements des mauvaises herbes du cadre de référence du robot au cadre mondial. Pour cela, vous devriez envisager d'obtenir les coordonnées GPS du robot et de les convertir en coordonnées cartésiennes dans le cadre mondial à l'aide du module **gps2cartesian** décrit dans [Task 1](https://parc-robotics.github.io/documentation-2023/fr/competition-instructions/phase-1/task-1-autonomous-field-navigation/#conversion-du-gps-en-cartesien).



## Transformer les ressources

Voici quelques ressources supplémentaires sur les transformations de coordonnées, en particulier dans le contexte de la robotique :

- [Transformations Partie 1 : Transformations de coordonnées et robotique](https://articulatedrobotics.xyz/transformations-1-coordinate_transforms/)
- [Transformations Partie 2 : Transformations linéaires](https://articulatedrobotics.xyz/transformations-2-linear_transforms/)
- [Transformations Partie 3 : Rotations 2D](https://articulatedrobotics.xyz/transformations-3-rotation_matrices_2d/)
- [Transformations Partie 4 : Traductions](https://articulatedrobotics.xyz/4-translations/)
- [Transformations Partie 5 : Matrices de transformation affine](https://articulatedrobotics.xyz/5-transformation_matrices/)
- [Transformations Partie 6 : Rotations 3D](https://articulatedrobotics.xyz/6-rotations_3d/)
- [Coordinate Transformations in Robotics - MathWorks](https://www.mathworks.com/help/robotics/ug/coordinate-transformations-in-robotics.html)
- [ROS Wiki : tf](https://wiki.ros.org/tf)
- [ROS Wiki : tf2](https://wiki.ros.org/tf2)
- [Accéder à l'arbre de transformation tf dans ROS](https://www.mathworks.com/help/ros/ug/access-the-tf-transformation-tree-in-ros.html)