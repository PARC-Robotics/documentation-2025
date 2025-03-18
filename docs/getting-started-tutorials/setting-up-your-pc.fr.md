# Configuration de votre PC

La **première étape** pour commencer le concours consiste à accéder à un ordinateur doté de la configuration système requise et à le configurer avec le système d'exploitation, les logiciels et l'environnement appropriés pour exécuter les packages du concours.

<!-- Ce guide vous aide à configurer votre ordinateur pour exécuter l'environnement de concurrence localement et développer le code.

Vous pouvez utiliser un ordinateur / ordinateur portable local ou une machine virtuelle dans votre ordinateur ou n'importe quelle plate-forme cloud comme [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc. (tous les fournisseurs de cloud ont un plan d'essai gratuit que vous pouvez utiliser). -->

## Configuration requise
Vous aurez besoin d’un ordinateur possédant toutes (ou au moins la plupart) de ces spécifications:
    
- un [gpu](https://en.wikipedia.org/wiki/graphics_processing_unit){target=_blank} dédié,
     - Les cartes Nvidia ont tendance à bien fonctionner à Ubuntu
- un processeur qui est au moins un Intel i5, ou équivalent,
- au moins 4 Go d'espace disque libre, 
- au moins 8 Go de RAM,

## Système d'exploitation (SE)

Nous utiliserons le système d'exploitation (SE) **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** qui est une variante de [Linux](https: //en.wikipedia.org/wiki/Linux){target=_blank}. Nous savons que certains participants peuvent avoir une expérience limitée ou nulle de l'utilisation d'Ubuntu. Voici donc un guide sur les différentes manières de configurer une instance Ubuntu Focal opérationnelle pour ce concours.

Si vous avez un PC Windows (ou tout autre système d'exploitation différent d'Ubuntu Jammy 22.04), voici deux (2) options à explorer :

- Option 1 (recommandée) : **Double démarrage** : installez **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** dans un démarrer à côté de votre système d'exploitation Windows.
<!-- - Option 2 : **Utiliser Docker :** Exécuter **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** dans un conteneur Docker sur votre Windows natif OS. -->
- Option 2 : **Utiliser une machine virtuelle :** Exécuter **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** dans une machine virtuelle (VM) sur votre système d'exploitation Windows natif.

<!-- !!! note
     Il est fortement recommandé d'installer [focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} d'Ubuntu en raison de [ROS (Noetic)](http://wiki.ros.org/noetic){target=_blank} dépendance.

## Installation de ROS
Vous devez installer ROS NOetic en suivant [ce guide](http://wiki.ros.org/noetic/installation/ubuntu){target=_blank} et installer `ros-nootic-desktop-full` dans l'étape` 1.4 `du guide. -->

=== "Double démarrage (recommandé)"
     - Voici un [bon guide](https://www.how2shout.com/linux/install-ubuntu-22-04-jammy-alongside-windows-10-dual-boot/){target=_blank} vous pouvez suivre pour installer **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}** dans un double démarrage avec votre système d'exploitation Windows.

=== "Utiliser une machine virtuelle"
     - Voici un [bon guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} vous pouvez suivre pour installer VirtualBox sur votre PC et exécuter Ubuntu.

<!-- === "Utiliser Docker" -->
<!--      - Nous avons fourni un [guide détaillé](../getting-started-tutorials/setting-up-with-docker.md) sur l'installation de Docker sur votre PC et la configuration du bon conteneur Docker pour exécuter toute la compétition. -->
<!--      - Si vous êtes préoccupé par l'option de double démarrage, nous vous recommandons d'envisager cette option. -->
<!---->
<!-- === "Utiliser une machine virtuelle" -->
<!--      - Voici un [bon guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} vous pouvez suivre pour installer VirtualBox sur votre PC et exécuter Ubuntu. -->
<!---->

## Installation de ROS
<!-- !!! note -->
<!-- Uncomment for Docker install -->
<!--      Si vous avez suivi l'option **Utiliser Docker** ci-dessus, veuillez IGNORER cette étape. -->
<!-- Une fois que vous avez une nouvelle installation **[d'Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}**, l'étape suivante consiste à installer ROS. Nous utilisons la distribution ROS2 Humble pour ce concours. Vous pouvez installer ROS2 Humble en suivant ce guide, installez la version Desktop, -->

Une fois que vous avez une nouvelle installation **[Ubuntu Jammy (22.04)](https://releases.ubuntu.com/jammy/){target=_blank}**, l'étape suivante consiste à installer ROS. Nous utilisons la distribution ROS2 Humble pour ce concours. Vous pouvez installer ROS2 Humble en suivant ce guide, installez la version Desktop,
`ros-humble-desktop`, et suivez le reste du guide de configuration. 

Si vous préférez les instructions vidéo, vous pouvez suivre cette vidéo :

- [Installer et configurer ROS 2 Humble](https://www.youtube.com/watch?v=0aPbWsyENA8){target=_blank}
