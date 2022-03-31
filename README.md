
# Projet Sauron
***
Le projet Sauron a été réalisé par les étudiants de 3ème année SRI de l'UPSSITECH dans le cadre de leur formation, en collaboration avec l'entreprise EXcent.   
Ce projet consiste en un système d'inspection de plaques de métal percées par un robot industriel. Il permet de localiser des trous sur une plaque et de vérifier leur bonne condition. Veuillez consulter le manuel de l'utilisateur et le manuel du programmeur pour plus de détails.   

L'application nécessite un ordinateur avec un environnement Linux et ROS Melodic présent.

## Installation des dépendances

En plus de ROS Melodic, le projet nécessite l'installation de certaines dépendances, en suivant les lignes de commandes ci-dessous.

Installation de *moveit* et *Industrial ROS* :
```
sudo apt install ros-melodic-industrial-core ros-melodic-moveit
```

Installation de *Pyquaternion* et *tsp-solver2* :
```
pip install pyquaternion
pip install tsp-solver2
```

Installation de *rosbridge* :
```
sudo apt-get install ros-melodic-rosbridge-server
```

## Installation du projet Sauron

Avant de lancer le projet en lui même, vous devez configurer l'environnement de travail. Il vous suffit d'ouvrir un nouveau terminal et de lancer les commandes suivantes :
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/PGE-UPSSITECH-2021/Projet_sauron.git
cd ~/catkin_ws
catkin_make
```

## Activer l'espace de travail

Si vous travaillez avec plusieurs espaces de travail, activez le nouvel espace de travail avec :
```
source ~/catkin_ws/devel/setup.bash
```
Si vous travaillez avec un seul espace de travail, vous pouvez utiliser :
```
echo "source  ~/catkin_ws/devel/setup.bash"  >>  ~/.bashrc 
source ~/.bashrc 
```

## Lancer le projet Sauron

Vous pouvez maintenant lancer le projet sauron à l'aide de la commande :
```
roslaunch deplacement_robot projet_sauron.launch
```
Enfin lancez l'interface graphique pour accéder aux manipulations.

## Cas où l'interface n'est pas disponible

Dans le cas où l'interface graphique n'est pas disponible, il est possible de lancer directement une action en utilisant le topic *message_ihm_run* et le message *deplacement_robot/msg/IHM_msg.msg* avec pour chaque paramètre les valeurs suivantes :

- *action* : un paramètre de cette liste : ["Localiser la plaque", "Identifier", "Verifier conformite", "Deplacement le robot", "Initialiser", "Calibrer"]
- *plaque* : un paramètre de cette liste : ["tole plate", "tole epaisse", "tole cintree"]
- *diametre* : une valeur de cette liste : ["5", "7", "12", "15"]
- *confiance* : nombre entre 0 et 100 sous forme de string
