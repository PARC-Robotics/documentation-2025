# En passant avec ROS

Le [Robot Operating System](https://www.ros.org/about-ros/) (ROS) est un cadre flexible pour l'écriture de logiciels de robot. Il s'agit d'un ensemble d'outils, de bibliothèques et de conventions qui visent à simplifier la tâche de création d'un comportement de robot complexe et robuste sur une grande variété de plates-formes robotiques.

Nous avons planifié ce concours autour du ROS en raison de ses fonctionnalités ainsi que de son utilisation généralisée dans la recherche et l'industrie en robotique. La version qui sera utilisée pour le concours de cette année est ROS 2 Humble et utilise Python.

![ROS et API](assets/ros-apis.png)

## Rampe de démarrage ROS 2!

Que vous soyez débutant ou développeur ROS 2 plus avancé, nous vous recommandons de prendre le temps de consulter les didacticiels ROS 2 Humble suivants, en particulier la section débutant.

* [Tutoriels officiels ROS 2](https://docs.ros.org/en/humble/Tutorials.html){target=_blank}
* [Liste de lecture Youtube du tutoriel ROS 2](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy){target=_blank} (**Fortement recommandé aux débutants**, le contenu est excellent!) 

!!! note
     Votre expérience d'apprentissage globale dans ce concours dépend ** fortement ** de la quantité de concepts fondamentaux de ROS 2 que vous pouvez comprendre dès le début. Par conséquent, nous vous ** recommandons fortement ** de consacrer le temps à utiliser ces ressources.


Ceci peut être effectué UNIQUEMENT après avoir configuré votre PC (en suivant le tutoriel ici : [Configuration de votre PC](../getting-started-tutorials/setting-up-your-pc.fr.md)).

## Écrire votre premier package ROS 2

Après avoir terminé les didacticiels requis répertoriés ci-dessus, vous pouvez commencer à [configurer l'espace de travail](../getting-started-tutorials/setting-up-your-workspace.fr.md).

En supposant que l'espace de travail à `~/ros2_ws/` est terminé à partir des étapes effectuées dans [configuration de votre espace de travail](../getting-started-tutorials/setting-up-your-workspace.fr.md),
ceci devrait être votre structure de dossiers:

```
~/ros2_ws/
├── build/
│   ├── .
│   └── .
├── install/
│   ├── .
│   └── .
├── log/
│   ├── .
│   └── .
└── src/
    ├── CMakeLists.txt
    └── PARC2025-Engineers-League/
        ├── parc_robot/
        │   ├── .
        │   ├── .
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── .
        └── .
```

Accédez d’abord au dossier source dans votre espace de travail,

```sh
cd ~/ros2_ws/src
```
Créez ensuite un nouveau package Python ROS 2 appelé `test_publisher` (par exemple) en exécutant la commande ci-dessous,

```shell
ros2 pkg create test_publisher --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs
```

Changez de répertoire dans le package ROS 2 Python nouvellement créé,

```shell
cd test_publisher/
```
La structure des fichiers du package `test_publisher` est la suivante,

```
├── package.xml
├── resource
│   └── test_publisher
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── test_publisher
    └── __init__.py
```

## Déplacer le robot programmatique

[Configuration de votre espace de travail](../getting-started-tutorials/setting-up-your-workspace.fr.md) a déjà montré comment contrôler le robot avec le clavier à l'aide de `teleop_twist_keyboard`.

Ce guide vous aidera à déplacer le robot en publiant des commandes dans le sujet `/robot_base_controller/cmd_vel_unstamped` par programmation en utilisant Python, qui est le langage qui sera utilisé dans le concours pour interagir avec ROS 2.

Pour ce faire, créez un fichier `robot_publisher.py` dans le répertoire `test_publisher` avec le fichier ``__init__`` de votre package ROS 2 (`test_publisher` dans ce cas) et rendez-le exécutable.

```shell
cd ~/ros2_ws/src/test_publisher/test_publisher
touch robot_publisher.py
chmod +x robot_publisher.py
```

!!! note 
    Vous devez modifier l'autorisation du fichier en exécutable pour pouvoir l'exécuter (comme cela a été fait dans la dernière commande ci-dessus).

Ouvrez maintenant le fichier et copiez et collez le code suivant à l'intérieur:

```python
#!/usr/bin/env python
"""
Script pour déplacer le robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        # Créez un éditeur qui peut "parler" au Robot et lui dire de se déplacer
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        # Créez un message de torsion et ajoutez des valeurs linéaires X et Z angulaires
        move_cmd = Twist()

        ######## Se déplacer tout droit ########
        print("Se déplaçant tout droit")
        move_cmd.linear.x = 0.5  # se déplacer en axe x à 0,5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # Pour les 4 secondes suivantes, publier les commandes cmd_vel move
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publier sur robot

        ######## Arrêt ########
        print("Arrêt")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0 # Donner à la fois zéro arrêtera le robot

        now = time.time()
        # Pour les 5 secondes suivantes, publier les commandes cmd_vel move
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotatif dans le sens intérieure ########
        print("Tournante")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # tourner à 0,7 rad/s

        now = time.time()
        # Pour les 15 secondes suivantes, publier les commandes cmd_vel move
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Arrêt ########
        print("Arrêt")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # Pour les 3 secondes suivantes, publier les commandes cmd_vel move
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Ce code fera bouger le robot tout droit pendant 4 secondes, s'arrêtera pendant 5 secondes, tournera dans le sens inverse des aiguilles d'une montre pendant 15 secondes, puis s'arrêtera.

## Compiler et exécuter

!!! Note 
    Nous devons mettre à jour le fichier `setup.py` dans le package ROS 2 pour inclure notre nouveau programme. Ajoutez la ligne suivante dans la section `console_scripts` du fichier `setup.py`:

    ```python
    entry_points={
            'console_scripts': [
                    'move_robot = test_publisher.robot_publisher:main',
            ],
    },
    ```
Exécutez les commandes suivantes pour compiler le code,

```shell
cd ~/ros2_ws
colcon build
```

Pour le voir fonctionner, exécutez d'abord le robot en simulation en exécutant la commande suivante dans un terminal,
```shell
source ~/ros2_ws/install/setup.bash
ros2 launch parc_robot_bringup task1_launch.py
```
Et exécutez les commandes suivantes dans un autre terminal pour exécuter ce nouveau programme,

```shell
source ~/ros2_ws/install/setup.bash
ros2 run test_publisher robot_publisher.py
```
Si vous avez tout bien configuré, vous devriez voir le robot se déplacer dans le Gazebo comme ci-dessous,

![publisher demo](assets/getting_started_demo.gif)

