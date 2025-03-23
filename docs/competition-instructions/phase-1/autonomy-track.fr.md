# Piste D'autonomie

## Description Générale

![task1_simulation](../assets/task1_sim.gif)

Les robots agricoles doivent être capables de se déplacer à travers les cultures et les terres agricoles, notamment en se déplaçant de manière autonome dans les rangs de maïs sur terrain accidenté. Cette tâche consiste à atteindre l'extrémité d'un rang, à effectuer un virage et à revenir dans les rangs adjacents jusqu'à l'objectif. Les équipes doivent développer un logiciel pour guider le robot le long d'un [chemin prédéfini](#explorer-plusieurs-mondes) au sein des rangs, de sa position de départ à son objectif.

## Instructions pour la tâche

### Lancement de la tâche
Dans un nouveau terminal, exécutez le fichier de lancement suivant pour lancer le robot dans Gazebo et RViz :

```sh
ros2 launch parc_robot_bringup task1_launch.py
```

Vous devriez voir l'affichage ci-dessous dans Gazebo et RViz respectivement. À droite, le robot et à gauche, le cercle vert qui représente l'emplacement cible.

=== "Gazebo"
    ![task1_gazebo](../assets/gazebo.png)

=== "RViz"
    ![task1_rviz](../../getting-started-tutorials/assets/rviz.png)

### Explorer Plusieurs Mondes

Nous avons préparé trois mondes que vous pouvez utiliser pour développer votre solution, chacun ayant une disposition et une longueur de ligne différentes.

=== "Monde 1"
    ![world1](../assets/World1.jpg)

=== "Monde 2"
    ![world2](../assets/World2.jpg)

=== "Monde 3"
    ![world3](../assets/World3.jpg)


La route par défaut est `world1`, mais vous pouvez sélectionner les deuxième et troisième options de route (`world2` et `world3`) en passant l'argument dans la commande `ros2 launch` comme suit :

```sh
## monde2
ros2 launch parc_robot_bringup task1_launch.py ​​world:=world2

## monde3
ros2 launch parc_robot_bringup task1_launch.py ​​world:=world3
```

### Obtenir la position de l'objectif GPS

Pour obtenir l'emplacement de l'objectif GPS pour cette tâche, quelle que soit l'option d'itinéraire, vous utilisez les [paramètres](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html){target=_blank} ROS dans un nœud.
   Voici un exemple de la façon d'obtenir l'emplacement de l'objectif en tant que paramètre ROS:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class GoalLocation(Node):
    def __init__(self):
        super().__init__("goal_location")

        # Déclarer les paramètres de latitude et de longitude de l'objectif
        self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        # Obtenir l'emplacement de l'objectif à partir du fichier yaml de coordonnées mondiales
        goal_lat = self.get_parameter("goal_latitude")
        goal_long = self.get_parameter("goal_longitude")

        # Imprimer l'emplacement de l'objectif
        self.get_logger().info(
            "goal location: %f %f"
            % (
                goal_lat.value,
                goal_long.value,
            )
        )


def main(args=None):
    rclpy.init(args=args)

    goal_location = GoalLocation()
    rclpy.spin(goal_location)

    goal_location.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
Les valeurs des paramètres d'emplacement de l'objectif sont obtenues à partir des fichiers `.yaml` fournis, qui varient en fonction de l'itinéraire choisi, et ces fichiers sont transmis comme arguments lorsque le nœud est exécuté comme suit:
`
ros2 run <nom_du_paquet> <nom_de_l'exécutable> --ros-args --params-file <nom_de_fichier>
`

Par exemple, en choisissant `route1` pour la tâche de navigation, l'exemple de commande suivant sera exécuté:

```sh

ros2 run <votre-nom-de-paquet> task1_solution.py --ros-args --params-file ~/ros2_ws/src/parc_robot_bringup/config/route1_world_coordinates.yaml
```

!!! note
    Un [chemin de fichier absolu](https://www.redhat.com/sysadmin/linux-path-absolute-relative){target=_blank} a été utilisé pour localiser le fichier de paramètres. Un chemin de fichier relatif peut également être utilisé si la commande est appelée dans le répertoire de `config` où se trouvent les fichiers de coordonnées. 
    Cependant, il est recommandé de spécifier le chemin absolu du fichier pour éviter les erreurs de chemin.

De même, les coordonnées GPS des piquets sur les terres agricoles peuvent être obtenues comme paramètre si vous en avez besoin pour la localisation. 
Pour obtenir les coordonnées du piquet A, par exemple, les paramètres `goal_latitude` et `goal_longitude` de l'extrait de code précédent sont remplacés respectivement par `peg_a_latitude` et `peg_a_longitude`.

A titre de référence, en considérant la `route1`, les coordonnées mondiales correspondantes sont disponibles dans le fichier [route1_world_coordinates.yaml](https://github.com/PARC-Robotics/PARC2024-Engineers-League/blob/main/parc_robot_bringup/config/route1_world_coordinates.yaml){target=_blank}.

!!! warning
    Veuillez **NE PAS** utiliser les coordonnées cartésiennes de l'emplacement du but et des piquets fournis par Gazebo ou le fichier mondial de quelque manière que ce soit. Vous serez pénalisé si vous le faites.

### Conversion du GPS en cartésien
Notre module, **gps2cartesian**, fournit un moyen pratique de convertir des emplacements GPS en coordonnées cartésiennes x-y. En utilisant l'origine mondiale du Gazebo comme origine de référence GPS (0, 0) en coordonnées cartésiennes, la fonction **gps_to_cartesian()** calcule les coordonnées cartésiennes de toute position GPS souhaitée transmise en paramètre à la fonction. Voici un exemple d'utilisation du module pour obtenir la coordonnée cartésienne du robot par rapport à l'origine de référence:

```python
#!/usr/bin/env python3
## Installez le module geographiclib 2.0 pour que ce code fonctionne.
## Pour installer geographiclib 2.0, copiez la ligne ci-dessous sur votre terminal.
## pip install geographiclib
## L'une des tâches de compétition PARC doit être en cours d'exécution pour que ce code fonctionne.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class GPSGoal(Node):
    def __init__(self):
        super().__init__("gps_goal")

        # Abonnez-vous une fois au sujet GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )

    def gps_callback(self, gps):

        # Récupérer les coordonnées cartésiennes à partir des coordonnées GPS
        x, y = gps_to_cartesian(gps.latitude, gps.longitude)

        # Imprimer les coordonnées cartésiennes
        self.get_logger().info(
            "La traduction de l'origine (0,0) à l'emplacement GPS fourni est: %.3f %.3f"
            % (x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    gps_goal = GPSGoal()
    rclpy.spin(gps_goal)

    gps_goal.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Préparer votre Solution

* Votre solution doit être préparée sous forme de packages ROS à enregistrer dans votre dossier de solutions. Créez un fichier exécutable de nœud dans votre package ROS qui exécute TOUT le code nécessaire à votre solution. Nommez ce fichier : `task1_solution.py`.

* Ainsi, votre solution à la tâche 1 doit être exécutée en appelant les commandes suivantes :

Dans un terminal :

```sh
ros2 launch parc_robot_bringup task1_launch.py
```

Ou

```sh
ros2 launch parc_robot_bringup task1_launch.py ​​world:=world2
```

Ou

```sh
ros2 launch parc_robot_bringup task1_launch.py ​​world:=world3
```

!!! note
    Veuillez patienter jusqu'à ce que les modèles du monde et du robot soient générés. Ce processus peut prendre plus de temps que d'habitude, surtout lors de la première exécution du programme.

Dans un autre terminal:

```sh
ros2 run <le-nom-de-votre-colis> task1_solution.py --ros-args --params-file <chemin-absolu-vers-la-route-coordonnées-mondiales-fichier-yaml>
```

## Règles de Tâche

* Le délai pour terminer la tâche est de **8 minutes (480 secondes)**.

* La tâche est terminée UNIQUEMENT lorsqu'UNE partie du robot se trouve à l'intérieur du cercle vert (marqueur d'emplacement de l'objectif) après avoir suivi le chemin prédéfini comme indiqué ci-dessus.

!!! note "Note"
    Assurez-vous de NE PAS fournir de solution avec des positions codées en dur vers lesquelles le robot doit se déplacer, car lors de l'évaluation, la position initiale du robot serait randomisée.

## Évaluation de l'autonomie

La notation de cette tâche repose sur les critères suivants :

| S/N | Critères/Indicateurs | Description |
| ----------- | ----------- | ------- |
| 1 | **Chemin prédéfini** | Suivez le chemin comme expliqué dans [Description du monde](#explorer-plusieurs-mondes). |
| 2 | **Évitement des plantes** | Le robot doit éviter tout contact avec les plants de maïs. **(Moins de contact, c'est mieux)** |
| 3 | **Distance finale jusqu'à l'objectif** | Distance la plus courte entre le robot (mesurée depuis son centre) et l'objectif à travers les rangs de culture, calculée à la limite de temps [8 minutes] **(Plus petite, mieux c'est)**
| 4 | **Temps de réalisation** | Temps écoulé entre le lancement de la solution et la réalisation de la tâche **(Plus petite, mieux c'est)** |
