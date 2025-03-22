# Écrire votre premier package ROS 2

En supposant que votre espace de travail, `~/ros2_ws/` dans ce cas, soit configuré conformément aux étapes de [configuration votre espace de travail](../getting-started-tutorials/setting-up-your-workspace.fr.md), voici la structure de vos dossiers :

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

Accédez d'abord au dossier source de votre espace de travail :
```shell
cd ~/ros2_ws/src
```

Créez ensuite un nouveau paquet Python ROS 2 appelé `test_publisher` (par exemple) en exécutant la commande suivante :
```shell
ros2 pkg create test_publisher --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs
```

Accédez au nouveau paquet Python ROS 2 :

```shell
cd test_publisher/
```

La structure du paquet `test_publisher` est la suivante :

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

## Déplacement du robot par programmation

Le guide [Configuration de votre espace de travail](../getting-started-tutorials/setting-up-your-workspace.fr.md) a déjà montré comment contrôler le robot au clavier grâce à `teleop_twist_keyboard`.

Ce guide vous aidera à déplacer le robot en publiant des commandes dans la rubrique `/robot_base_controller/cmd_vel_unstamped` par programmation Python.

Pour ce faire, créez un fichier `robot_publisher.py` dans le répertoire `test_publisher`, contenant le fichier `__init__`, et rendez-le exécutable.

```shell
cd ~/ros2_ws/src/test_publisher/test_publisher
touch robot_publisher.py
chmod +x robot_publisher.py
```

!!! note
    Vous devez modifier les permissions du fichier pour qu'il soit exécutable (comme indiqué dans la dernière commande ci-dessus).

Ouvrez maintenant le fichier et copiez-collez le code suivant :


```python
#!/usr/bin/env python3
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
        # Créez un éditeur qui peut « parler » au robot 
        # et lui dire de se déplacer
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        # Créez un message Twist et ajoutez des valeurs x 
        # linéaires et z angulaires
        move_cmd = Twist()

        ######## Aller tout droit ########
        print("Aller tout droit")
        move_cmd.linear.x = 0.5  # se déplacer sur l'axe x à 0,5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # Pendant les 4 prochaines secondes, publiez les commandes 
        # de déplacement cmd_vel
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publier sur robot
            
        ######## Arrêt ########
        print("Arrêt")
        # Affecter les deux valeurs à 0,0 arrête le robot.
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0 

        now = time.time()
        # Pendant les 5 prochaines secondes, publiez les commandes 
        # de déplacement cmd_vel
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotation dans le sens inverse 
        # des aiguilles d'une montre ########
        print("Rotation")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotation à 0,7 rad/s.

        now = time.time()
        # Pendant les 15 prochaines secondes, publiez les commandes 
        # de déplacement cmd_vel
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Arrêt ########
        print("Arrêt")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # Pendant les 3 prochaines secondes, publiez les commandes 
        # de déplacement cmd_vel
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Sortie")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Ce code fera bouger le robot en ligne droite pendant 4 secondes, s'arrêtera pendant 5 secondes, tournera dans le sens inverse des aiguilles d'une montre pendant 15 secondes, puis s'arrêtera.

## Compiler et exécuter

!!! note 
    Nous devons mettre à jour le fichier « setup.py » du package ROS 2 pour inclure notre nouveau programme. Ajoutez la ligne suivante dans la section `console_scripts` du fichier `setup.py` :

    ```python
    entry_points={
            'console_scripts': [
                    'move_robot = test_publisher.robot_publisher:main',
            ],
    },
    ```

Exécutez les commandes suivantes pour compiler le code :

```shell
cd ~/ros2_ws
colcon build
```

Pour voir le fonctionnement, exécutez d'abord le robot en simulation en exécutant la commande suivante dans un terminal :

```shell
source ~/ros2_ws/install/setup.bash
ros2 launch parc_robot_bringup task1_launch.py
```

Et exécutez les commandes suivantes dans un autre terminal pour exécuter ce nouveau programme :

```shell
source ~/ros2_ws/install/setup.bash
ros2 run test_publisher robot_publisher.py
```

Si vous avez bien configuré tout, vous devriez voir le robot se déplacer dans Gazebo comme ci-dessous.

![publisher demo](assets/getting_started_demo.gif)