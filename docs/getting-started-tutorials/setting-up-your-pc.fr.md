# Configuration de votre PC

La **première étape** pour démarrer la compétition consiste à accéder à un ordinateur avec la configuration système requise et à le configurer avec le système d'exploitation, le logiciel et l'environnement appropriés pour exécuter les packages de compétition.

## Configuration requise

Vous aurez besoin d'un ordinateur possédant toutes (ou au moins la plupart) les spécifications suivantes :

- Un [processeur graphique](https://en.wikipedia.org/wiki/Graphics_processing_unit){target=_blank} dédié 
    - Les cartes graphiques Nvidia fonctionnent généralement bien sous Ubuntu
- Un processeur Intel i5 ou équivalent
- Au moins 4 Go d'espace disque disponible
- Au moins 8 Go de RAM

## Système d'exploitation (SE)

Nous utiliserons le système d'exploitation (SE) **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}**, une variante de [Linux](https://en.wikipedia.org/wiki/Linux){target=_blank}. Nous savons que certains participants peuvent avoir une expérience limitée, voire inexistante, d'Ubuntu. Voici donc un guide expliquant comment configurer une instance Ubuntu Noble Numbat opérationnelle pour ce concours.

Si vous possédez un PC Windows (ou tout autre système d'exploitation différent d'Ubuntu Noble Numbat 24.04), voici deux (2) options à explorer :

- Option 1 (recommandée) : **Dual-Boot** : Installez **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** en double démarrage parallèlement à votre système d'exploitation Windows.

- Option 2 : **Utilisation d'une machine virtuelle** : Exécutez **[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** dans une machine virtuelle (VM) sur votre système d'exploitation Windows natif.

=== "Dual-Boot (recommandé)"
- Cette [vidéo](https://www.youtube.com/watch?v=qq-7X8zLP7g){target=_blank} explique pas à pas le processus de double démarrage d'**[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/jammy/){target=_blank}** avec votre système d'exploitation Windows. Ce [guide](https://linuxconfig.org/how-to-install-ubuntu-alongside-windows-11-dual-boot){target=_blank} permet également d'obtenir la même configuration de double démarrage.

=== "Utilisation d'une machine virtuelle"
- L'installation de VirtualBox sur votre PC pour exécuter Ubuntu est décrite dans cette [vidéo](https://www.youtube.com/watch?v=kSy3NX3Pe-c){target=_blank}. Vous pouvez également suivre ce [guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} pour installer VirtualBox et exécuter Ubuntu.


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

Une fois votre installation d'**[Ubuntu Noble Numbat (24.04)](https://releases.ubuntu.com/noble/){target=_blank}** terminée, l'étape suivante consiste à installer ROS. Nous utilisons la distribution [ROS2 Jazzy](https://docs.ros.org/en/jazzy) pour ce concours. Vous pouvez installer ROS2 Jazzy en suivant [ce guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html){target=_blank}, installer la version Desktop, `ros-jazzy-desktop`, et suivre le reste du guide d'installation.

Si vous préférez les instructions vidéo, vous pouvez regarder cette [vidéo YouTube](https://www.youtube.com/watch?v=oTp2UtHZLH0){target=_blank} (un article de blog est inclus dans la description de la vidéo).